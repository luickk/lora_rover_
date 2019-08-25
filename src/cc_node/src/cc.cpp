#include <ros/ros.h>

#include "std_msgs/String.h"

#include "driving_node/move_side.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cc");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);

    // ros::ServiceClient client = n.serviceClient<driving_node::move_side>("move_side");
    // driving_node::move_side srv;
    // srv.request.side = "left";
    // srv.request.dir = "forward";
    // srv.request.throttle = 50;
    //
    // if (client.call(srv))
    // {
    //    ROS_INFO("called srv");
    // }

    ros::spin();

}
