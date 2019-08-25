#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>

#include "gps_nav_node/turn_to.h"
#include "gps_nav_node/nav_to.h"

bool nav_to(gps_nav_node::nav_to::Request  &req, gps_nav_node::nav_to::Response &res)
{
  return true;
}

long get_live_lat(){

}

bool turn_to(gps_nav_node::turn_to::Request  &req, gps_nav_node::turn_to::Response &res)
{
  long to_lat = req.lat;

  long to_lon = req.lon;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Driving Node");
  ros::NodeHandle n;

  ros::ServiceServer serv1 = n.advertiseService("turn_to", turn_to);
  ros::ServiceServer serv2 = n.advertiseService("nav_to", nav_to);

  ros::spin();

  return 0;
}
