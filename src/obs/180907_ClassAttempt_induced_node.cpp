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
	ros::NodeHandle nh;


	void static callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl, const WindStampedConstPtr& wind);

	public:
		CorrData();
		~CorrData();
};
CorrData::CorrData()
{

	message_filters::Subscriber<Imu> imu_sub(nh, "/mavros/imu/data", 100);
	message_filters::Subscriber<Odometry> gpl_sub(nh, "/mavros/global_position/local", 100);
 	message_filters::Subscriber<WindStamped> wind_sub(nh, "/wind_data", 100);

 	typedef sync_policies::ApproximateTime<Imu, Odometry, WindStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, gpl_sub, wind_sub);
  	sync.registerCallback(boost::bind(&callback, _1, _2, _3));


}


void callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl, const WindStampedConstPtr& wind)
{
  // Solve all of perception here...
	ROS_INFO("---------------------------------");
	ROS_INFO("IMU Seq# is %i",imu->header.seq);
	ROS_INFO("IMU OmegaX is %f deg/s",imu->angular_velocity.x);
	ROS_INFO("GPL Seq# is %i",gpl->header.seq);
	ROS_INFO("GPL VeloX is %f m/s",gpl->twist.twist.linear.x);
	ROS_INFO("U velocity is %f m/s",wind->u);

	ROS_INFO("IMU time is %i sec",imu->header.stamp.sec);
	ROS_INFO("GPL time is %i sec",gpl->header.stamp.sec);
	ROS_INFO("WIND time is %i sec",wind->header.stamp.sec);

	ROS_INFO("IMU time is %i nsec",imu->header.stamp.nsec);
	ROS_INFO("GPL time is %i nsec",gpl->header.stamp.nsec);
	ROS_INFO("WIND time is %i nsec",wind->header.stamp.nsec);

	uint imu_time = imu->header.stamp.nsec;
	uint wind_time = wind->header.stamp.nsec;
	int del_sonic_imu = (imu_time - wind_time);
	double del_sonic_imu_f = del_sonic_imu * 0.000000001;
	ROS_INFO("Anemometer Time difference: %f nsec",del_sonic_imu_f);

ros::NodeHandle n;
ros::Publisher wind_pub_corr = n.advertise<WindStamped>("wind_data_corrected",5);
ros::Time induced_start_time = ros::Time::now();
WindStamped msg;

Vector3d P12(0.08, 0, 0.66);
Matrix3d R12, R0S, R21, RBA, R2B, R2A, RE0, RE2;
R12 << 1,0,0,
   	   0,1,0,
   	   0,0,1;
R0S << 0,1,0,
       1,0,0,
       0,0,-1;
Vector3d Omega11(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z);

Quaterniond Q01(gpl->pose.pose.orientation.w,gpl->pose.pose.orientation.x,gpl->pose.pose.orientation.y,gpl->pose.pose.orientation.z);
Matrix3d R01 = Q01.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix

Vector3d VS1(gpl->twist.twist.linear.x,gpl->twist.twist.linear.y,gpl->twist.twist.linear.z);
Matrix3d R1S = R01.transpose() * R0S;
Vector3d V11 = R1S * VS1;

R21 = R12.transpose();
Vector3d V22 = R21 * (V11 + Omega11.cross(P12));

//Rotation matrix about x of 45 degrees from MATLAB
RBA << 1,0,0,
	 0,0.7071,-0.7071,
     0,0.7071,0.7071;

R2B << 0,-1,0,
       0,0,1,
       -1,0,0;
R2A = RBA*R2B;      
Vector3d DAwind(wind->u,wind->v,wind->w); 
Vector3d D2wind = R2A * DAwind;
Vector3d D2corrwind = D2wind + V22;
RE0 = R0S;
RE2 = RE0*(R01*R12);
cout << "matrix is " <<endl<< R01 << endl;
Vector3d DEcorrwind = RE2 *	D2corrwind;
cout << "corrected wind: "<<endl << DEcorrwind << endl;

msg.header.seq = wind->header.seq;
msg.header.stamp = induced_start_time;
msg.header.frame_id = '0';
msg.u = DEcorrwind[0]; //u
msg.v = DEcorrwind[1]; //v
msg.w = DEcorrwind[2]; //w
msg.temp = wind->temp;
wind_pub_corr.publish(msg);

}

int main(int argc, char** argv)
{
	ROS_INFO("Starting");
	ros::init(argc, argv, "induced_node");

	CorrData *corr_wind;
	corr_wind = new CorrData();


	ROS_INFO("Subscribers Made");
	
    while(ros::ok)
    {
      ROS_INFO(" before ros spin()");
      ros::spin();
      ROS_INFO(" after ros spin()");
    
    }
    delete(corr_wind);
 	return 0;
}