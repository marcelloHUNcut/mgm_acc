#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "nav_msgs/Path.h"

struct acc_pub
{
    

};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"acc_node");
    ros::NodeHandle(acc_node);

    ros::spin();

    return 0;

}

