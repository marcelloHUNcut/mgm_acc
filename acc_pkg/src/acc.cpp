#include "ros/ros.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


struct acc_pub
{
    acc_pub(ros::NodeHandle nh_):n(nh_)
    {
       scanin=n.subscribe("/agent2/scan", 1, &acc_pub::scan_callback, this);
       speedoutpub=n.advertise<ackermann_msgs::AckermannDriveStamped>("/agent2/ackermann_cmd",1);
       speedoutpub2=n.advertise<ackermann_msgs::AckermannDriveStamped>("/agent1/ackermann_cmd",1);
       speedin=n.subscribe("/agent2/odom/ground_truth", 1, &acc_pub::scan_callback3, this);
       speedin2=n.subscribe("/agent1/odom/ground_truth", 1, &acc_pub::scan_callback2, this);
    } 
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {

        if((scan_in->range_min < scan_in->ranges[0]) && (scan_in->ranges[0] < scan_in->range_max)){

            // TODO: Filter
            calculate_distance(scan_in->ranges[0]);
            

        }
        
       
    }
    void scan_callback2(const nav_msgs::Odometry &odom){

        speed=odom.twist.twist.linear.x;
        
    }
    void scan_callback3(const nav_msgs::Odometry &odom2){

        speed2=odom2.twist.twist.linear.x;
        
    }
    double timetocollision(double relativespeed, double distance)
    {
        float timetocollison=distance/relativespeed;
        return timetocollison;
    }

    double sec_dist(double min_dist, double speed_)
    {
        // min_dist= speed2 *dt_min;
        // min_dist = std::max(min_dist, 0.1);
        if(speed2<=0.2)
        {
            min_dist=0.5;
        }
        else
        {
            if (speed2>0.2 & speed2<2.0)
            {
                min_dist=1.0;
            }
            
        }
        
        return min_dist;
    }

    void calculate_distance(double distance){       

        if(speed>=0.05)
        {
            if(distance < sec_dist(min_dist,speed2))
            {
                // ToDO rethink
                speed2-=0.1;

            }
            else
            {
                speed2>0;
                speed2=1.0;
            }
            if(distance > sec_dist(min_dist,speed2))
            {
                speed2+=0.1;
                //speedlimiter
                if (speed2>=0.7)
                {
                    speed2=0.7;
                }
                
                
            }  
        }
        else
        {
            speed2=0.0;
                      
        }
        //no negative speed
        //speed2 = std:min(std::max(0.0, speed2), desired_speed);
        if(speed2<=0)
        {
            speed2=0;
        }
        if (timetocollision(relativespeed,distance)>=3.0)
        {
            speed2=speed2;
        }
        else
        {
            speed2-=0.2;
        }
        
        
       
        
        backspeed.drive.speed = speed2;
        speedoutpub.publish(backspeed);
   
    }
    
    
    double min_dist = 0.05;
    double max_dist=5.0;
    double speed=0;
    double speed2=0;
    double relativespeed=abs(speed2-speed);

    ackermann_msgs::AckermannDriveStamped backspeed;
    ackermann_msgs::AckermannDriveStamped backspeed2;

    ros::Subscriber scanin;
    ros::Publisher speedoutpub;
    ros::Publisher speedoutpub2;
    ros::Subscriber speedin;
    ros::Subscriber speedin2;
    ros::NodeHandle n;
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"acc_node");
    ros::NodeHandle nh("~");
    acc_pub x(nh);
    ros::spin();
}

