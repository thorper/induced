#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;


//const sensor_msgs::Imu msg

void callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl)
{
  // Solve all of perception here...
	ROS_INFO("---------------------------------");
	ROS_INFO("IMU Seq# is %i",imu->header.seq);
	ROS_INFO("IMU OmegaX is %f deg/s",imu->angular_velocity.x);
	ROS_INFO("GPL Seq# is %i",gpl->header.seq);
	ROS_INFO("GPL VeloX is %f m/s",gpl->twist.twist.linear.x);
	// ROS_INFO(" ");
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "induced_node");

	ros::NodeHandle nh;
  
	message_filters::Subscriber<Imu> imu_sub(nh, "/mavros/imu/data", 50);
	message_filters::Subscriber<Odometry> gpl_sub(nh, "/mavros/global_position/local", 50);
 
	typedef sync_policies::ApproximateTime<Imu, Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, gpl_sub);
  	sync.registerCallback(boost::bind(&callback, _1, _2));





	ros::spin();

 	return 0;
}