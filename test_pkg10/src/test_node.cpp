#include "ros/ros.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Path.h"

struct scan2pcl{

    scan2pcl(ros::NodeHandle nh_):nh(nh_){
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
        dist_pub = nh.advertise<std_msgs::Float64MultiArray>("/distances", 1);
        path_pub = nh.advertise<nav_msgs::Path>("/high_dist", 1);
        scan_sub = nh.subscribe("/agent1/scan", 1, &scan2pcl::scan_callback, this);

        geometry_msgs::PoseStamped pose;
        path_out.poses.push_back(pose);
        path_out.poses.push_back(pose);

    }

    // callback for laser scan
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        // reset
        max_dist = 0.0;
        distances.data.clear();

        // Get the points from the scan message
        pcl::PointCloud<pcl::PointXYZ> localCloud;
        for (int i=0; i<scan_in->ranges.size(); i++)
        {
            //Convert to cartesian coordinates
            if(scan_in->ranges[i] > scan_in->range_min && scan_in->ranges[i] < scan_in->range_max){
                float angle = scan_in->angle_min + i*scan_in->angle_increment;

                pcl::PointXYZ newPoint;
                newPoint.x = scan_in->ranges[i] * cos(angle);
                newPoint.y = scan_in->ranges[i] * sin(angle);
                newPoint.z = 0;

                // Get distances between points
                if (localCloud.points.size()>0){
                    calculate_distance(localCloud.back(), newPoint);
                }

                localCloud.points.push_back(newPoint);
            }
        }

        // Get distances between points
        if (localCloud.points.size()>2){
            calculate_distance(localCloud.back(), localCloud.front());
        }

        // Conversion of pcl/PointCloud to sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(localCloud,temp);
        temp.header.frame_id = scan_in->header.frame_id;
        
        // publish point cloud
        cloud_pub.publish(temp);

        // publish distances
        dist_pub.publish(distances);

        // publish path
        path_out.header = scan_in->header;
        path_pub.publish(path_out);
    }

    void calculate_distance(pcl::PointXYZ point1, pcl::PointXYZ point2){
        float dist = sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2));
        distances.data.push_back(dist);

        if(dist > max_dist){
            max_dist = dist;

            path_out.poses[0].pose.position.x = point1.x;
            path_out.poses[0].pose.position.y = point1.y;
            path_out.poses[0].pose.position.z = point1.z;

            path_out.poses[1].pose.position.x = point2.x;
            path_out.poses[1].pose.position.y = point2.y;
            path_out.poses[1].pose.position.z = point2.z;

        }

    }

    double max_dist = 0.0;
    
    std_msgs::Float64MultiArray distances;
    nav_msgs::Path path_out;

    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    ros::Publisher dist_pub;
    ros::Publisher path_pub;
    ros::NodeHandle nh;
};

int main(int a, char** aa) {
	ros::init(a, aa, "scan2pcl");
	ros::NodeHandle n("~");

    scan2pcl scan_converter(n);

    ros::spin();

}