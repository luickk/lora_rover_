#include "ros/ros.h"

#include "std_msgs/String.h"

#include "compass_node/compass_raw.h"

#include "qmc5883l.cpp"

#include <wiringPiI2C.h>
#include <wiringPi.h>

#include <sstream>
#include <iostream>
#include <limits>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compass_publisher");

  ros::NodeHandle n;
  ros::Rate rate(1);

  compass_node::compass_raw compass_data;

  ros::Publisher chatter_pub = n.advertise<compass_node::compass_raw>("compass_raw", 1);

  RoverQMC5883L compass;

  compass_node::compass_raw comp_raw;

  compass.initialize();
  float heading;
  while (ros::ok())
  {
    heading = compass.read();
    comp_raw.dir=heading;
    chatter_pub.publish(comp_raw);
    //ROS_INFO("compass: %f", heading);
    rate.sleep();
  }

  ros::spin();

  return 0;
}
