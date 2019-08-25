#include "ros/ros.h"

#include "std_msgs/String.h"
#include "gps_node/gps_raw.h"


#include "MicroNMEA.cpp"

#include <wiringSerial.h>
#include <wiringPi.h>

#include <sstream>
#include <iostream>
#include <limits>

int main(int argc, char **argv)
{
  int fd;

  ros::init(argc, argv, "gps_publisher");

  ros::NodeHandle n;

  gps_node::gps_raw gps_data;

  ros::Publisher chatter_pub = n.advertise<gps_node::gps_raw>("gps_raw", 1);

  ros::Rate loop_rate(1000);

	if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
  }
  REINIT:if ((fd = serialOpen ("/dev/ttyS0", 9600)) < 0)
  {
   fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  }

  //MicroNMEA library structures
  char nmeaBuffer[200];
  MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

  std::string data;
  std::string NavSystem;
  int NumSatellites;
  long lat;
  long lon;

  int input = 0;
  while (ros::ok())
  {
    while (serialDataAvail (fd))
    {
      input = serialGetchar (fd);

      nmea.process(input);

      //data.push_back(input);

      NumSatellites = nmea.getNumSatellites();

      lat = nmea.getLatitude();
      lon = nmea.getLongitude();


      if (lat > 1 && lon > 1)
      {
        lat = stoi(to_string(lat).insert(2, "."));
        lon = stoi(to_string(lon).insert(1, "."));
      }

      //std::cout << "NumSatellites: " << NumSatellites << ", ";
      //std::cout << "Lat: " << lat << ", ";
      //std::cout << "Lon: " << lon << std::endl;

      //std::cout << data;

      gps_data.num_sats = NumSatellites;
      gps_data.lat = lat;
      gps_data.lon = lon;

      chatter_pub.publish(gps_data);

      //data = "";
      ros::spinOnce();
      loop_rate.sleep();
      if(input==-1){
        goto REINIT;
      }
    }
  }

  ros::spin();

  return 0;
}
