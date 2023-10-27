#include "ros/ros.h"
#include <iostream>
#include <deque>                                                     
#include <numeric>    

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct filter{

    filter(ros::NodeHandle nh_):nh(nh_){
        avg_pub = nh.advertise<std_msgs::Float64>("/avg_dist", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mid_point", 1);

        path_sub = nh.subscribe("/high_dist", 1, &filter::path_callback, this);
        dist_sub = nh.subscribe("/distances", 1, &filter::dist_callback, this);

        timer = nh.createTimer(ros::Duration(1.0), &filter::timer_callback, this);


    }

    // callback for distances
    void dist_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        auto dists = msg->data;
        auto avg_dist = std::accumulate(dists.begin(), dists.end(),0.0) / dists.size();
        avg_distances.push_back(avg_dist);
        if (avg_distances.size() > 10){
            avg_distances.pop_front();
        }
    }

    //callback for path 
    void path_callback(const nav_msgs::Path::ConstPtr& msg)
    {
        auto mid_point = geometry_msgs::PoseStamped();
        // get position of mid point
        mid_point.pose.position.x = (msg->poses[0].pose.position.x + msg->poses[1].pose.position.x) / 2;
        mid_point.pose.position.y = (msg->poses[0].pose.position.y + msg->poses[1].pose.position.y) / 2;
        mid_point.pose.position.z = (msg->poses[0].pose.position.z + msg->poses[1].pose.position.z) / 2;

        // get orientation of mid point perpendiculat to the path
        auto dx = msg->poses[1].pose.position.x - msg->poses[0].pose.position.x;
        auto dy = msg->poses[1].pose.position.y - msg->poses[0].pose.position.y;

        double angle = atan2(dy, dx) + M_PI / 2;

        // convert angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        tf2::convert(q, mid_point.pose.orientation);


        mid_point.header = msg->header;
        pose_pub.publish(mid_point);
    }

    // callback of timer
    void timer_callback(const ros::TimerEvent& event)
    {
        if (avg_distances.size() > 0 ){

            auto avg_dist = std_msgs::Float64();
            avg_dist.data = std::accumulate(avg_distances.begin(), avg_distances.end(), 0.0) / avg_distances.size();
            avg_pub.publish(avg_dist);
        }
    }

    std::deque<double> avg_distances;

    ros::Timer timer;
    ros::Subscriber dist_sub;
    ros::Subscriber path_sub;
    ros::Publisher avg_pub;
    ros::Publisher pose_pub;
    ros::NodeHandle nh;
};

int main(int a, char** aa) {
	ros::init(a, aa, "filter");
	ros::NodeHandle n("~");

    filter filtering(n);

    ros::spin();

}