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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compass_publisher");

  ros::NodeHandle n;
  ros::Rate rate(1);

  compass_node::compass_raw compass_data;

  ros::Publisher chatter_pub = n.advertise<compass_node::compass_raw>("compass_raw", 1);
  // python /home/pi/lora_rover/src/compass_node/src/qmc5883l.py
  // chatter_pub.publish(comp_raw);

  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("python /home/pi/lora_rover/src/compass_node/src/qmc5883l.py", "r"), pclose);

  if (!pipe) {
      throw std::runtime_error("popen() failed!");
  }

  compass_node::compass_raw comp_raw;

  double heading;

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
      heading = std::stod(result);
      //std::cout << heading << std::endl;
      comp_raw.dir = heading;
      chatter_pub.publish(comp_raw);
  }

  ros::spin();

  return 0;
}
