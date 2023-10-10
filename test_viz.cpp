#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"

struct viz_pub{

    viz_pub(ros::NodeHandle nh_):n(nh_)
    {
        std::string odom_topic_name;
        n.param<std::string>("odom_topic_name", odom_topic_name, "/odom");
        pub = n.advertise<visualization_msgs::MarkerArray>("/agent1/viz", 1000);
        sub = n.subscribe(odom_topic_name, 100, &viz_pub::callback_odom, this);

    }

    void callback_odom(const nav_msgs::Odometry::ConstPtr msg){
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = msg->pose.pose;

        marker.scale.x = 0.3;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.4;
        marker.color.b = 0.4;

        marker.lifetime = ros::Duration(0.1);

        marker_array.markers.push_back(marker);

        pub.publish(marker_array);

    }

    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "test_viz");

    ros::NodeHandle nh("~");

    viz_pub viz_publisher(nh);

    ros::spin();
}
