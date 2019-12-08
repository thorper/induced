#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;


//const sensor_msgs::Imu msg
class windData
{
	int fd;

	void callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl);

	public:   
	    windData();
	    ~windData(){close(fd);} //properly closes port
};
//----------------Constructor----------------------------
windData::windData()
{
    fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(fd == -1)            // Error Checking 
           printf("\n  Error! in Opening ttyUSB0  ");
    else
           printf("\n  ttyUSB0 Opened Successfully ");
}
//------------- CALLBACK function to receive item_order data---------------
void windData::callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl)
{
	// Solve all of perception here...
	ROS_INFO("---------------------------------");
	ROS_INFO("IMU Seq# is %i",imu->header.seq);
	ROS_INFO("IMU OmegaX is %f deg/s",imu->angular_velocity.x);
	ROS_INFO("GPL Seq# is %i",gpl->header.seq);
	ROS_INFO("GPL VeloX is %f m/s",gpl->twist.twist.linear.x);	
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "induced_node");

	ros::NodeHandle nh;

	windData *wind_imu; //pointer to a windData class object
	wind_imu = new windData(); //instantiates memory in the heap

  
	message_filters::Subscriber<Imu> imu_sub(nh, "/mavros/imu/data", 50);
	message_filters::Subscriber<Odometry> gpl_sub(nh, "/mavros/global_position/local", 50);
 
	typedef sync_policies::ApproximateTime<Imu, Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, gpl_sub);
  	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	delete(wind_imu); //deletes windData object

 	return 0;
}