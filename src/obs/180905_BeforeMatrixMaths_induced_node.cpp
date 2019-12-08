#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <induced/WindStamped.h>
// #include </home/thorper/catkin_ws/devel/include/induced/WindStamped.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace induced;


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

	

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "induced_node");
ROS_INFO("Starting");
	ros::NodeHandle nh;

	message_filters::Subscriber<Imu> imu_sub(nh, "/mavros/imu/data", 100);
	message_filters::Subscriber<Odometry> gpl_sub(nh, "/mavros/global_position/local", 100);
 	message_filters::Subscriber<WindStamped> wind_sub(nh, "/wind_data", 100);

ROS_INFO("Subscribers Made");
	typedef sync_policies::ApproximateTime<Imu, Odometry, WindStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, gpl_sub, wind_sub);
  	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

ROS_INFO("Ros Spin start");
	ros::spin();
ROS_INFO("Ros Spin done");
 	return 0;
}