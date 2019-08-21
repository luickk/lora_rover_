#include "ros/ros.h"

#include "std_msgs/String.h"
#include "gps_node/gps_raw.h"

#include <wiringSerial.h>
#include <wiringPi.h>

#include <sstream>


int main(int argc, char **argv)
{
  int fd;

  ros::init(argc, argv, "gps_publisher");

  ros::NodeHandle n;

  gps_node::gps_raw gps_data;

  ros::Publisher chatter_pub = n.advertise<gps_node::gps_raw>("gps_raw", 1);

  ros::Rate loop_rate(1000);



	if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }
  REINIT:if ((fd = serialOpen ("/dev/ttyAMA0", 9600)) < 0)
  {
   fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  }

  int input = 0;
  while (ros::ok())
  {
    while (serialDataAvail (fd))
    {
      input = serialGetchar (fd);


      //ROS_DEBUG(input);

      //chatter_pub.publish(gps_data);

      ros::spinOnce();
      loop_rate.sleep();
      if(input==-1){
        goto REINIT;
      }
    }
  }
  return 0;
}
