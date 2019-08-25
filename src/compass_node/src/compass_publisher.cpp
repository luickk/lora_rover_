#include "ros/ros.h"

#include "std_msgs/String.h"

#include "compass_node/compass_raw.h"

#include <wiringPiI2C.h>
#include <wiringPi.h>

#include <sstream>
#include <iostream>
#include <limits>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compass_publisher");

  ros::NodeHandle n;

  compass_node::compass_raw compass_data;

  ros::Publisher chatter_pub = n.advertise<compass_node::compass_raw>("compass_raw", 1);

  ros::Rate loop_rate(1000);

  int fd, result;

  fd = wiringPiI2CSetup(0x1E);

  // std::cout << "Init result: "<< fd << std::endl;

  while (ros::ok())
  {
    result = wiringPiI2CRead (fd);

    //std::cout << result << std::endl;

    //compass_data.dir = result;

    //chatter_pub.publish(compass_data);

    ros::spinOnce();
    loop_rate.sleep();
    if(result == -1)
    {
      //std::cout << "Error.  Errno is: " << errno << std::endl;
    }
  }
  ros::spin();

  return 0;
}
