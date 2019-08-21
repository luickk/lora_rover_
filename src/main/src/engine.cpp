#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "engine");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);
    ROS_DEBUG("Main Node started");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
