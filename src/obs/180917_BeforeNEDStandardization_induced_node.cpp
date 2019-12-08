#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <induced/WindStamped.h>

#include <Eigen/Dense>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace induced;
using namespace Eigen;
using namespace std;


class CorrData
{

	public:

		Matrix3d R12, R0S, R21, RE0, RE2;
		
		CorrData()
		{
			imu_sub.subscribe(nh,"/mavros/imu/data", 50);
			gpl_sub.subscribe(nh,"/mavros/global_position/local", 50);
			wind_sub.subscribe(nh, "/wind_data", 100);

			wind_pub_corr = nh.advertise<WindStamped>("wind_data_corrected",50);

 			sync.reset(new Sync(MySyncPolicy(100),imu_sub, gpl_sub, wind_sub));
  			sync->registerCallback(boost::bind(&CorrData::callback,this,_1, _2, _3));

		}

		void callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl, const WindStampedConstPtr& wind)
		{
			ROS_INFO("Induced Start");
			// ros::Time induced_start_time = ros::Time::now();
			WindStamped msg;
			
			// Matrix3d R12, R0S, R21, RBA, R2B, R2A, RE0, RE2;
			
			// 1. Rotation matrix about x-axis of 45 degrees from MATLAB (moved into wind node)
			// RBA << 1,0,0,
			// 	 0,0.7071,0.7071,
			//      0,-0.7071,0.7071; //Moved to wind node
			
			// R2B << 0,-1,0,
			//        0,0,1,
			//        -1,0,0;
			// R2A = R2B*RBA;  

			//2. Set Orientation/position of Anemometer wrt Pixhawk
			Vector3d P12(-0.088, 0, 0.876); // 9/17/18 P12(-0.08, 0, 0.66);


			// 3. Axis of 1 as viewed from 2  
			R12 << 1,0,0,
			   	   0,1,0,
			   	   0,0,1;     
			R21 = R12.transpose();

			// 4/5. Read in Gyro Data (steps 4 and 5 in MATLAB. Ignoring {G} since {G}={1})
			Vector3d Omega11(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z);
			// Vector3d Omega11(imu->angular_velocity.x,0,0); //only consider x angular velocity

			//Vector3d Omega11(0,0,0); //Ignore angular velocities

			//6. Transforms between earth an velocity frames
			R0S << 0,1,0,
			       1,0,0,
			       0,0,-1;
			RE0 = R0S;

			//7. Read in linear velocity
			Vector3d VS1(gpl->twist.twist.linear.x,gpl->twist.twist.linear.y,gpl->twist.twist.linear.z);

			//8a. Generate rotation matrix from quaternions
			Quaterniond Q01(gpl->pose.pose.orientation.w,gpl->pose.pose.orientation.x,gpl->pose.pose.orientation.y,gpl->pose.pose.orientation.z);
			// Matrix3d R01 = Q01.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
			Matrix3d R01; //Ignore orientation of UAV
			R01  << 1,0,0,
					0,1,0,
					0,0,1;

			//8b. s1000 linear velocity wrt 1 
			Matrix3d R1S = R01.transpose() * R0S;
			// Vector3d V11 = R1S * VS1; //Consider linear velocity
			Vector3d V11(0,0,0); //Ignore linear Velocity of UAV

			//8c. Instantaneous velocity of 2 wrt 2
			Vector3d V22 = R21 * (V11 + Omega11.cross(P12));
			
			//9. Read in Anemometer data from A frame   
			// Vector3d DAwind(wind->u,wind->v,wind->w); //wind data now comes in {2} not {A}
			Vector3d D2wind(wind->u,wind->v,wind->w); 
			// Vector3d D2wind(0,wind->v,0); // only read the y,v component of wind


			//10a. converting wind vector to NWU frame  and account for 45 degree rotation
			// Vector3d D2wind = R2A * DAwind; //de in step 9

			//10b. Induced velocity "subtraction"
			Vector3d D2corrwind = D2wind + V22;

			//11. Find matrix to convert to 'regular' earth frame
			RE2 = RE0*(R01*R12);

			//12. Final conversion to earth frame
			Vector3d DEcorrwind = RE2 *	D2corrwind;

			// cout << "matrix is " <<endl<< R01 << endl;
			// cout << "corrected wind: "<<endl << DEcorrwind << endl;
			
			msg.header.seq = wind->header.seq;
			msg.header.stamp = wind->header.stamp;
			msg.header.frame_id = '0';
			// msg.u = DEcorrwind[0]; //u
			// msg.v = DEcorrwind[1]; //v
			// msg.w = DEcorrwind[2]; //w
			msg.u = D2corrwind[0]; //u
			msg.v = D2corrwind[1]; //v
			msg.w = D2corrwind[2]; //w
			msg.temp = wind->temp;
			wind_pub_corr.publish(msg);
			
			// ros::Time induced_end_time = ros::Time::now();
			// ros::Duration corr_dur = induced_end_time - induced_start_time;
			// double sec_corr_dur = corr_dur.toSec();
			//ROS_INFO("Time to correct is: %f secs",sec_corr_dur);
			//cout << corr_dur <<endl;
			ROS_INFO("Induced End");
		}

	private:
		ros::Publisher wind_pub_corr;
		ros::NodeHandle nh;
		message_filters::Subscriber<Imu> imu_sub;
		message_filters::Subscriber<Odometry> gpl_sub;
 		message_filters::Subscriber<WindStamped> wind_sub;

		typedef sync_policies::ApproximateTime<Imu, Odometry, WindStamped> MySyncPolicy;
		typedef	Synchronizer<MySyncPolicy> Sync;
		boost::shared_ptr<Sync> sync;
};


int main(int argc, char** argv)
{
	ROS_INFO("Starting Correction Node");
	
	ros::init(argc, argv, "induced_node");

	CorrData corr_wind;

    ros::spin();
  
 	return 0;
}