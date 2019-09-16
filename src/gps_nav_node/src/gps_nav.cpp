#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <cmath>

#include "gps_nav_node/turn_to.h"
#include "gps_nav_node/nav_to.h"
#include "gps_node/gps_raw.h"

#include "driving_node/move_side.h"
#include "compass_node/compass_raw.h"

# define PI		3.14159265358979323846	/* pi */

float toRad(float degree) {
    return degree/180 * PI;
}
float toDeg(float radian) {
    return radian * 180/PI;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {

	// Convert degrees to radians
	lat1 = lat1 * M_PI / 180.0;
	lon1 = lon1 * M_PI / 180.0;

	lat2 = lat2 * M_PI / 180.0;
	lon2 = lon2 * M_PI / 180.0;

	// radius of earth in metres
	double r = 6378100;

	// P
	double rho1 = r * cos(lat1);
	double z1 = r * sin(lat1);
	double x1 = rho1 * cos(lon1);
	double y1 = rho1 * sin(lon1);

	// Q
	double rho2 = r * cos(lat2);
	double z2 = r * sin(lat2);
	double x2 = rho2 * cos(lon2);
	double y2 = rho2 * sin(lon2);

	// Dot product
	double dot = (x1 * x2 + y1 * y2 + z1 * z2);
	double cos_theta = dot / (r * r);

	double theta = acos(cos_theta);

	// Distance in Metres
	return r * theta;
}

/*
  Local function to read gps location
*/
gps_node::gps_raw get_latest_gps_data()
{
  gps_node::gps_raw latest_gps_data = *ros::topic::waitForMessage<gps_node::gps_raw>("/gps_raw", ros::Duration(10));

  return latest_gps_data;
}

/*
  Local function to get compass dir
*/
int get_latest_dir()
{
  compass_node::compass_raw latest_dir = *ros::topic::waitForMessage<compass_node::compass_raw>("/compass_raw", ros::Duration(10));

  return latest_dir.dir;
}

// Returns true if x is in range [low..high], else false
bool in_Range(unsigned low, unsigned high, unsigned x)
{
    return  ((x-low) <= (high-low));
}
double _ftod(float fValue)
{
    char czDummy[30];
    sprintf(czDummy,"%9.5f",fValue);
    double dValue = strtod(czDummy,NULL);
    return dValue;
}

int calc_heading(float lat,float lon,float lat2,float lon2){
	lat2=(lat2*3.14159)/180;
	lon2=(lon2*3.14159)/180;
	lat=(lat*3.14159)/180;
	lon=(lon*3.14159)/180;
	float londif=lon2-lon;

	float head=atan2((sin(londif)*cos(lat2)),((cos(lat)*sin(lat2))-(sin(lat)*cos(lat2)*cos(londif)))) ;
	float finalans=(head*180)/3.14159;
	float finalans2=0;
	int final_heading;
	if(finalans<=0)
	{
		finalans2=finalans+360;
		final_heading= finalans2;
	} else {
		final_heading= finalans;
	}
	return final_heading;
}

bool nav_to(gps_nav_node::nav_to::Request  &req, gps_nav_node::nav_to::Response &res)
{
  ROS_INFO("navto init");
  gps_node::gps_raw latest_gps_data = get_latest_gps_data();

  float to_lat = req.lat;

  float to_lon = req.lon;

  float live_lat = latest_gps_data.lat;

  float live_lon = latest_gps_data.lon;

	// debug:

	//float live_lat = 49.466611;

	//float live_lon = 10.967948;

	float distance_to_dest;



  if(to_lat != 0 && to_lon != 0 && live_lat != 0 && live_lon != 0)
  {
		int final_heading = calc_heading(live_lat, live_lon, to_lat, to_lon);

		// done calc heading
		ROS_INFO("DRIVING TO POS  lat: %f, lon: %f FROM lat: %f lon: %f", to_lat, to_lon, live_lat, live_lon);
    ROS_INFO("Calc Heading: %d", final_heading);

		gps_nav_node::turn_to turn;
		turn.request.dir=final_heading;
		if (ros::service::call("driving_node/move_side", turn)){}


		distance_to_dest = calculateDistance(live_lat, live_lon, to_lat, to_lon);
		ROS_INFO("DISTANCE TO DEST: %f", distance_to_dest);

    driving_node::move_side move;
    move.request.dir="forward";
    move.request.side="both";
    move.request.throttle=50;
    if (ros::service::call("move_side", move)){}

    while(ros::ok())
    {
      latest_gps_data = get_latest_gps_data();
      distance_to_dest = calculateDistance(latest_gps_data.lat, latest_gps_data.lon, to_lat, to_lon);
			ROS_INFO("DISTANCE (LEFT) TO DEST: %f", distance_to_dest);
      if(distance_to_dest < 20){
          // TODO | stop drive forward
          move.request.throttle=0;
          if (ros::service::call("move_side", move)){}
      }
    }
  }
  res.status = 1;
  return true;
}

bool turn_to(gps_nav_node::turn_to::Request  &req, gps_nav_node::turn_to::Response &res)
{
	ROS_INFO("Turning to %f", req.dir);
	int live_heading;

	int exec_deg = req.dir;
	driving_node::move_side move;
	move.request.dir="forward";
	move.request.side="left";
	move.request.throttle=50;
	if (ros::service::call("move_side", move)){}
	move.request.dir="backward";
	move.request.side="right";
	if (ros::service::call("move_side", move)){}

	while(ros::ok()){
		live_heading = get_latest_dir();
		ROS_INFO("%d", live_heading);
		if(in_Range(exec_deg-20,exec_deg+20, (uint)live_heading)){
			move.request.side="both";
			move.request.throttle=0;
			if (ros::service::call("move_side", move)){}
			break;
		}
	}
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
