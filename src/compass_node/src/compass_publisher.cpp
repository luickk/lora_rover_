#include "ros/ros.h"

#include "std_msgs/String.h"

#include "compass_node/compass_raw.h"

#include "qmc5883l.cpp"

#include <wiringPiI2C.h>
#include <wiringPi.h>

#include <sstream>
#include <iostream>
#include <limits>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include "process.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compass_publisher");

  ros::NodeHandle n;
  ros::Rate rate(1);

  compass_node::compass_raw compass_data;

  ros::Publisher chatter_pub = n.advertise<compass_node::compass_raw>("compass_raw", 1);
  // python /home/pi/lora_rover/src/compass_node/src/qmc5883l.py
  // chatter_pub.publish(comp_raw);

  while(ros::ok())
  {
    // requires py-qmc5883l lib: https://github.com/RigacciOrg/py-qmc5883l
    procxx::process ping( "python", "/home/pi/lora_rover/src/compass_node/src/qmc5883l.py");
    ping.exec();

    std::string line;
    std::getline( ping.output(), line);
    // std::cout << line << std::endl;
    compass_data.dir = std::stoi(line);
    chatter_pub.publish(compass_data);
  }

  ros::spin();

  return 0;
}
