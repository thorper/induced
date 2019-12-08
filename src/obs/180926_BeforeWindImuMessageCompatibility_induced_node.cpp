#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <induced/WindStamped.h>
#include <induced/WindStampedPose.h>

#include <Eigen/Dense>

// using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace induced;
using namespace Eigen;
using namespace std;


class CorrData
{

	public:

		Matrix3d R12, R21, RM1, R1M, RE0, R0M;

		
		CorrData()
		{
			lpo_sub.subscribe(nh,"/mavros/local_position/odom", 50);

			wind_sub.subscribe(nh, "/wind_data", 100);

			wind_pub_corr = nh.advertise<WindStampedPose>("wind_data_corrected_pose",50);

 			sync.reset(new Sync(MySyncPolicy(100),lpo_sub, wind_sub));
  			sync->registerCallback(boost::bind(&CorrData::callback,this,_1, _2));

		}

		void callback(const OdometryConstPtr& lpo, const WindStampedConstPtr& wind)
		{
			//ROS_INFO("Induced Start");
			// ros::Time induced_start_time = ros::Time::now();
			WindStampedPose msg;
				
			/*1. Calculate induced velocity V22*/	
				//1.a Coordinate Axis of 1 frame as viewed from 2 frame 
					R12 << 1,0,0,
					   	   0,1,0,
					   	   0,0,1; 
					R21 = R12.transpose();
				//1.b Calculate linear velocity of UAV in its own frame V11
					//Simple transform to change from Pixhawk FCU frame 1 to ROS FCU frame M
					RM1 << 1,0,0,
						   0,-1,0,
						   0,0,-1;
					R1M = RM1.transpose();
					//Read in linear velocity and convert to V11
					Vector3d VM1(lpo->twist.twist.linear.x,lpo->twist.twist.linear.y,lpo->twist.twist.linear.z);
					Vector3d V11 = R1M * VM1; //Consider linear velocity
				//1.c Calculate angular velocity of UAV in its own frame Omega11
					Vector3d OmegaM1(lpo->twist.twist.angular.x,lpo->twist.twist.angular.y,lpo->twist.twist.angular.z);		
					Vector3d Omega11 = R1M*OmegaM1;
				//1.d Set Orientation/position of Anemometer wrt Pixhawk
					Vector3d P12(-0.088, 0, -0.876); // 9/17/18 P12(-0.08, 0, 0.66);
				Vector3d V22 = R21 * (V11 + Omega11.cross(P12));

			/*2. Calculate corrected wind velocity in anemometer 2 frame, not global wind speed yet...*/
				//2.a Read in raw wind speed values in 2 frame
					Vector3d D2raw(wind->u,wind->v,wind->w); 
				Vector3d D2corr = D2raw + V22;

			/*3. Transform corrected wind data to earth frame, E frame*/
				//3.a Simple ROS earth {0}(ENU) to Aviation/Pixhawk earth {E}(NED)
					RE0 << 0,1,0,
				   		   1,0,0,
				           0,0,-1;
				//3.b Generate rotation matrix from quaternion
				/*
				The story of what this pesky quaternion really means...
				First, everything is referenced from the ROS 0 frame. The 0 frame is ENU meaning the +x points east.
				Now ROS places the UAV frame, the M frame, with its +x also facing due east. If the UAV spins for
				example +90 degrees about the 0's Z axis (The UAV no points north) then the quaterion will report
				that a rotation of +90 about Z0 (the z from the 0 frame) took place. So if you point the UAV North
				to start the quaternion will not convert to euler XYZ angles of 0,0,0 but it will convert to 0,0,90
				*/
					Quaterniond Q0M(lpo->pose.pose.orientation.w,lpo->pose.pose.orientation.x,lpo->pose.pose.orientation.y,lpo->pose.pose.orientation.z);
					R0M = Q0M.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
				Vector3d DEcorr = (RE0*(R0M*(RM1*(R12*D2corr))));

			//Debug
			ROS_INFO("OmegaM1");
			ROS_INFO("%f, %f, %f",lpo->twist.twist.angular.x,lpo->twist.twist.angular.y,lpo->twist.twist.angular.z);
			ROS_INFO("Q0M components: w,x,y,z");
			ROS_INFO("%f, %f, %f, %f",lpo->pose.pose.orientation.w,lpo->pose.pose.orientation.x,lpo->pose.pose.orientation.y,lpo->pose.pose.orientation.z);
			ROS_INFO("R0M matrix is");
			cout << R0M << endl;
			

			// Publish to ROS
			msg.header.seq = wind->header.seq;
			msg.header.stamp = wind->header.stamp;
			msg.header.frame_id = 'E';
			msg.u = DEcorr[0]; //u
			msg.v = DEcorr[1]; //v
			msg.w = DEcorr[2]; //w

			msg.temp = wind->temp;
			msg.x = lpo->pose.pose.position.x;
			msg.y = lpo->pose.pose.position.y;
			msg.z = lpo->pose.pose.position.z;
			wind_pub_corr.publish(msg);

			/*string dda = "/dev/ttyUSB0";
			string da;
    		nh.param("device_address",da,dda);
    		ROS_INFO("Look here for anemometer: %s",da.c_str());
			int sr;
			nh.param("sample_rate",sr,50);
			ROS_INFO("Integer from Launch file is: %i",sr);*/
		}

	private:
		ros::Publisher wind_pub_corr;
		ros::NodeHandle nh;

		message_filters::Subscriber<Odometry> lpo_sub;
 		message_filters::Subscriber<WindStamped> wind_sub;

		typedef sync_policies::ApproximateTime<Odometry, WindStamped> MySyncPolicy;
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