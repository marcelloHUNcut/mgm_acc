#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub_cloud;
ros::Publisher pub_viz;



void callback_scan(const sensor_msgs::LaserScan msg)
{ 
    //pontfelhobe konvertalas
    pcl::PointCloud<pcl::PointXYZ> localCloud;
    for (int i = 0; i < msg.ranges.size(); i++)
    {
        float angle=msg.angle_min+i*msg.angle_increment;
        if (msg.range_min< msg.ranges[i] && msg.ranges[i]<msg.range_max)
        {
            pcl::PointXYZ newpoint;
            newpoint.x=msg.ranges[i]*cos(angle);
            newpoint.y=msg.ranges[i]*sin(angle);
            newpoint.z=msg.ranges[i];


            localCloud.points.push_back(newpoint);
        }
        
    }
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(localCloud, cloud);
    cloud.header=msg.header;
    pub_cloud.publish(cloud);
    double x=0;
    double y=0;
    double z=0;
    for (int j = 0; j < localCloud.points.size(); j++)
    {
        x+=localCloud.points[j].x;
        y+=localCloud.points[j].y;
        z+=localCloud.points[j].z;
    }
    
    double avg_x=x/localCloud.points.size();
    double avg_y=y/localCloud.points.size();
    double avg_z=z/localCloud.points.size();
    visualization_msgs::Marker mark;
    mark.header=msg.header;
    mark.ns="weight_point";
    mark.id=0;
    mark.type=visualization_msgs::Marker::SPHERE;
    mark.action=visualization_msgs::Marker::ADD;
   
    mark.pose.position.x=avg_x;
    mark.pose.position.y=avg_y;
    mark.pose.position.z=avg_z;
    mark.color.a=1.0;
    mark.color.r=1.0;
    mark.color.g=1.0;
    mark.color.b=1.0;

    mark.scale.x=0.1; //méterben van értve
    mark.scale.y=0.1;
    mark.scale.z=0.1;

    pub_viz.publish(mark);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "kiskutya2");
    ros:: NodeHandle nh("~");
    ros::Subscriber sub=nh.subscribe("/agent1/scan",1,callback_scan);
    ros::spin();

    pub_cloud =nh.advertise<sensor_msgs::PointCloud2>("/cloud",1);
    pub_viz=nh.advertise<visualization_msgs::Marker>("/marker",1);
}