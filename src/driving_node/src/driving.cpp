#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>

#include "driving_node/move_side.h"

int d1_m1 = 22;
int d2_m1 = 23;

int d1_m2 = 24;
int d2_m2 = 25;

int pwm_m1 = 28;
int pwm_m2 = 29;

bool move_side(driving_node::move_side::Request  &req, driving_node::move_side::Response &res)
{
  ROS_INFO("service called srv init");
  int throttle = req.throttle;
  std::string dir = req.dir;

  wiringPiSetup();

  pinMode(d1_m1,OUTPUT);
  pinMode(d2_m1,OUTPUT);

  pinMode(d1_m2,OUTPUT);
  pinMode(d2_m2,OUTPUT);;

  softPwmCreate(pwm_m1,0,100);
  softPwmCreate(pwm_m2,0,100);

  if(req.side=="left")
  {
    if(dir=="forward"){
      softPwmWrite(pwm_m2, throttle);
      digitalWrite(d2_m2, 0);
      digitalWrite(d1_m2, 1);
      ROS_INFO("turning left; forward");
    } else if(dir=="backward"){
      softPwmWrite(pwm_m2, throttle);
      digitalWrite(d2_m2, 1);
      digitalWrite(d1_m2, 0);
      ROS_INFO("turning left; backward");
    }
  }
  else if(req.side=="right")
  {
    if(dir=="forward"){
      softPwmWrite(pwm_m1, throttle);
      digitalWrite(d2_m1, 1);
      digitalWrite(d1_m1, 0);
      ROS_INFO("turning right; forward");
    } else if(dir=="backward"){
      softPwmWrite(pwm_m1, throttle);
      digitalWrite(d2_m1, 0);
      digitalWrite(d1_m1, 1);
      ROS_INFO("turning right; backward");
    }
  }
  else if(req.side=="both")
  {
    if(dir=="forward"){
      softPwmWrite(pwm_m1, throttle);
      digitalWrite(d2_m1, 1);
      digitalWrite(d1_m1, 0);

      softPwmWrite(pwm_m2, throttle);
      digitalWrite(d2_m2, 0);
      digitalWrite(d1_m2, 1);
      ROS_INFO("turning both; forward");
    } else if(dir=="backward"){
      ROS_INFO("turning both; backward");
      softPwmWrite(pwm_m1, throttle);
      digitalWrite(d2_m1, 0);
      digitalWrite(d1_m1, 1);

      softPwmWrite(pwm_m2, throttle);
      digitalWrite(d2_m2, 1);
      digitalWrite(d1_m2, 0);
    }
  }
  res.status = 1;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Driving Node");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);

  ros::ServiceServer service = n.advertiseService("move_side", move_side);

  ros::spin();

  return 0;
}
