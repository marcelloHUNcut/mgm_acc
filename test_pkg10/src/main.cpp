#include "ros/ros.h"
#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>

struct scan2pcl{
        
    scan2pcl(ros::NodeHandle nh_):n(nh_)
    {
        cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
        dist_pub = n.advertise<std_msgs::Float64MultiArray>("/distances",1);
        path_pub = n.advertise<nav_msgs::Path>("/high_dist",1);
        scan_sub = n.subscribe("/agent1/scan", 1, &scan2pcl::scan_callback, this);

        geometry_msgs::PoseStamped point;
        path_out.poses.push_back(point);
        path_out.poses.push_back(point);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

        pcl::PointCloud<pcl::PointXYZ> localCloud;
        for(int i = 0;i < scan_in->ranges.size();i += 5){
            if((scan_in->range_min < scan_in->ranges[i]) && (scan_in->ranges[i] < scan_in->range_max)){
                float angle = scan_in->angle_min + i * scan_in->angle_increment;

                //laser scan pointcloudda konvertalasa
                pcl::PointXYZ newPoint;
                newPoint.x = scan_in->ranges[i] * cos(angle);   
                newPoint.y = scan_in->ranges[i] * sin(angle);
                newPoint.z = 0;

                if(localCloud.points.size()>0){
                    calculate_distance(localCloud.points.back(), newPoint);
                }
                
                localCloud.points.push_back(newPoint);
            }
        }
        if(localCloud.points.size()>2){
            calculate_distance(localCloud.points.back(), localCloud.points.front());
        }

        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(localCloud, temp);

        temp.header.frame_id = "map";

        cloud_pub.publish(temp);

        dist_pub.publish(distances);

        path_out.header = scan_in->header;
        path_pub.publish(path_out);
    }

    void calculate_distance(pcl::PointXYZ point1, pcl::PointXYZ point2){
        double dist = sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
        distances.data.push_back(dist);

        if(dist > max_dist){
            max_dist = dist;
            path_out.poses[0].pose.position.x = point1.x;
            path_out.poses[0].pose.position.y = point1.y;

            path_out.poses[1].pose.position.x = point2.x;
            path_out.poses[1].pose.position.y = point2.y;

        }
    }

    std_msgs::Float64MultiArray distances;
    double max_dist = 0.0;
    nav_msgs::Path path_out;

   
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    ros::Publisher dist_pub;
    ros::Publisher path_pub;
    ros::NodeHandle n;
};

int main(int a, char** aa) {

    ros::init(a, aa, "scan2pcl");
    ros::NodeHandle nh("~");
    scan2pcl scan_converter(nh);

    ros::spin();
}