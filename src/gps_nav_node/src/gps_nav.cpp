#include "ros/ros.h"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#include "gps_nav_node/turn_to.h"
#include "gps_nav_node/nav_to.h"
#include "gps_node/gps_raw.h"
#include "driving_node/move_side.h"

double calc_distance(double lat1, double lon1, double lat2, double lon2) {

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

bool nav_to(gps_nav_node::nav_to::Request  &req, gps_nav_node::nav_to::Response &res)
{
  long to_lat = req.lat;

  long to_lon = req.lon;

  gps_node::gps_raw latest_gps_data = get_latest_gps_data();

  long live_lat = latest_gps_data.lat;

  long live_lon = latest_gps_data.lon;

  if(to_lat != 0 && to_lon != 0 && live_lat != 0 && live_lon != 0)
  {
    gps_node::gps_raw latest_gps_data = get_latest_gps_data();

    // calc heading
    float begin_lat1, begin_lon1, begin_lat2, begin_lon2;
    begin_lat1 = live_lat;
    begin_lon1 = live_lon;
    begin_lat2 = to_lat;
    begin_lon2 = to_lon;
    float londif, head, lon1, lon2, lat1, lat2, finalans, finalans2, final_heading;
    lat2=(begin_lat2*3.14159)/180;
    lon2=(begin_lon2*3.14159)/180;
    lat1=(begin_lat1*3.14159)/180;
    lon1=(begin_lon1*3.14159)/180;
    londif=lon2-lon1;

    ROS_INFO("DRIVING TO POS  lat: %f, lon: %f FROM lat: %f lon: %f", to_lat, to_lon, begin_lat1, begin_lon1);

    head=atan2((sin(londif)*cos(lat2)),((cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(londif)))) ;
    finalans=(head*180)/3.14159;
    finalans2=0;
    if(finalans<=0)
    {
      finalans2=finalans+360;
      final_heading= finalans2;
    } else {
      final_heading= finalans;
    }
    // done calc heading
    ROS_INFO("Calc Heading: %f",final_heading);
    gps_node::gps_raw latest_gps_data_sync;


    // TODO | CALL TURN_TO
    gps_nav_node::turn_to turn;
    turn.request.dir=final_heading;
    if (ros::service::call("move_side", turn)){}

    double distance_to_dest;
    distance_to_dest = calc_distance(_ftod(live_lat), _ftod(live_lon), _ftod(to_lat), _ftod(to_lon));
    ROS_INFO("Calc Dist: %lf",distance_to_dest);


    // TODO | CALL drive forward
    driving_node::move_side move;
    move.request.dir="forward";
    move.request.side="both";
    move.request.throttle=50;
    if (ros::service::call("move_side", move)){}

    while(1)
    {
      latest_gps_data_sync = get_latest_gps_data();
      distance_to_dest = calc_distance(_ftod(latest_gps_data_sync.lat), _ftod(latest_gps_data_sync.lon), _ftod(to_lat), _ftod(to_lon));
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
