#include <ros/ros.h>

//#include "driving_node/move_side.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "engine");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    // ROS_DEBUG("Main Node started");
    //
    // ros::ServiceClient client = n.serviceClient<engine_node::move_side>("move_side");
    // engine_node::move_side srv;
    // srv.request.side = "left";
    // srv.request.dir = "forward";
    // srv.request.throttle = 50;
    //
    // if (client.call(srv))
    // {
    //
    // }

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
