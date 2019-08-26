#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lora Com Node");
  ros::NodeHandle n;

  ros::spin();

  return 0;
}
