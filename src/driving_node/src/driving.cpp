#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>

#include "driving_node/move_side.h"

int d1_m1 = 22;
int d2_m1 = 23;

int d1_m2 = 24;
int d2_m2 = 25;

int pwm_m1 = 28;
int pwm_m2 = 29;

bool move_side(driving_node::move_side::Request  &req, driving_node::move_side::Response &res)
{
  int throttle = req.throttle;
  std::string dir = req.dir;

  pinMode(d1_m1,OUTPUT);
  pinMode(d2_m1,OUTPUT);

  pinMode(d1_m2,OUTPUT);
  pinMode(d2_m2,OUTPUT);
  pinMode(pwm_m2,OUTPUT);

  // softPwmCreate(pwm_m1,0,50);
  // softPwmCreate(pwm_m2,0,50);

  if(req.side=="right")
  {
    pwmWrite(d1_m2, 50);
    digitalWrite(d2_m2, 1);
    digitalWrite(pwm_m2, 0);
    std::cout << "turning right" << std::endl;
  }
  else if(req.side=="left")
  {
    pwmWrite(d1_m1, 50);
    digitalWrite(d2_m1, 1);
    digitalWrite(pwm_m1, 0);
    std::cout << "turning left" << std::endl;
  }
  else if(req.side=="both")
  {
    std::cout << "turning both" << std::endl;
    pwmWrite(d1_m1, 50);
    digitalWrite(d2_m1, 1);
    digitalWrite(pwm_m1, 0);

    pwmWrite(d1_m2, 50);
    digitalWrite(d2_m2, 1);
    digitalWrite(pwm_m2, 0);

  }
  res.status = 1;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Driving Node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("move_side", move_side);

  ros::spin();

  return 0;
}
