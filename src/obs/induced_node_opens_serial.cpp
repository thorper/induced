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


	//-------------------------------Termios Settings Start----------------------------------
 	//Setting the Port Settings
  	//www.cmrr.umn.edu/~strupp/serial.html#2_3_1
  	struct termios SerialPortSettings;

  	tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
  	cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed as 19200                       
  	cfsetospeed(&SerialPortSettings,B57600); // Set Write Speed as 9600                             
  
	// No parity (8N1):
  	SerialPortSettings.c_cflag &= ~PARENB;
  	SerialPortSettings.c_cflag &= ~CSTOPB;
  	SerialPortSettings.c_cflag &= ~CSIZE;
  	SerialPortSettings.c_cflag |= CS8;                               

  	// SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
  	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
  
   	// SerialPortSettings.c_iflag |= (IXON | IXOFF | IXANY);// Enable XON/XOFF flow control both i/p and o/p 
  	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);// Disable XON/XOFF flow control both i/p and o/p 
    
  	// SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);// raw mode                             
  	SerialPortSettings.c_iflag |= (ICANON | ECHO | ECHOE | ISIG);// Cannonical mode                             

  	SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing

  	// Setting Time outs
  	SerialPortSettings.c_cc[VMIN] = 0; // Read at least 10 characters 21
  	SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  ERROR ! in Setting attributes");
	else
		printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 
	//----------------------------------Termios Settings End-------------------------------------

}
//------------- CALLBACK function to receive item_order data---------------
void callback(const ImuConstPtr& imu, const OdometryConstPtr& gpl)
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
