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
       scanin = n.subscribe("/agent2/scan", 1, &acc_pub::scan_callback, this);
       speedout_rear = n.advertise<ackermann_msgs::AckermannDriveStamped>("/agent2/ackermann_cmd",1);
       speedout_front = n.advertise<ackermann_msgs::AckermannDriveStamped>("/agent1/ackermann_cmd",1);
       speedin_rear = n.subscribe("/agent2/odom/ground_truth", 1, &acc_pub::scan_callback_rear, this);
       speedin_front = n.subscribe("/agent1/odom/ground_truth", 1, &acc_pub::scan_callback_front, this);
       odometry_sub_front = n.subscribe("/agent1/odom/ground_truth", 1, &acc_pub::odometryCallback_front, this);
       odometry_sub_rear = n.subscribe("/agent2/odom/ground_truth", 1, &acc_pub::odometryCallback_rear, this);

    } 
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {

        double relative_speed = speed_front - speed_rear;
        double relative_accel = accel_front - accel_rear;
       
        if((scan_in->range_min < scan_in->ranges[0]) && (scan_in->ranges[0] < scan_in->range_max)){

            // TODO: Filter
            distance = scan_in->ranges[0];
        
        }
        else{
            distance = 33;
        }


        time_to_collision(distance, relative_speed, relative_accel);
        acc(speed_rear, speed_desired);

        ROS_INFO("ttc: %f", ttc);
        ROS_INFO("distance: %f", distance);
        //ROS_INFO("speed_rear: %f", speed_rear);
        ROS_INFO("accel_rear: %f", accel_rear);
        //ROS_INFO("speed_front: %f", speed_front);
        //ROS_INFO("accel_front: %f", accel_front);
        ROS_INFO("relative_speed %f", relative_speed);
        ROS_INFO("relative_accel %f", relative_accel);

    }

    void scan_callback_front(const nav_msgs::Odometry &odom_front){

        //speed_front = odom_front.twist.twist.linear.x;
        speed_front = 0.1;

    }
    
    void scan_callback_rear(const nav_msgs::Odometry &odom_rear){

        speed_rear = odom_rear.twist.twist.linear.x;
        
    }

    void odometryCallback_front(const nav_msgs::Odometry::ConstPtr& msg) {
        if (lastOdometry_front != nullptr) {
            ros::Time currentTime = ros::Time::now();
            double deltaTime = (currentTime - lastOdometry_front->header.stamp).toSec();

            double linearVelocity = lastOdometry_front->twist.twist.linear.x;
            double test_front = msg->twist.twist.linear.x - linearVelocity;
            double accel_front = test_front / deltaTime;
            //ROS_INFO("deltaTime_front: %f", deltaTime);
            //ROS_INFO("linearVelocity_front: %f", linearVelocity);
            //ROS_INFO("test_front %f", test_front);
        }

        lastOdometry_front = msg;
    }

    void odometryCallback_rear(const nav_msgs::Odometry::ConstPtr& msg) {
        if (lastOdometry_rear != nullptr) {
            ros::Time currentTime = ros::Time::now();
            double deltaTime = (currentTime - lastOdometry_rear->header.stamp).toSec();

            double linearVelocity = lastOdometry_rear->twist.twist.linear.x;
            double test_rear = msg->twist.twist.linear.x - linearVelocity;
            double accel_rear = test_rear / deltaTime;
            //ROS_INFO("deltaTime_rear: %f", deltaTime);
            //ROS_INFO("linearVelocity_rear: %f", linearVelocity);
            //ROS_INFO("test_rear: %f", test_rear);
            //ROS_INFO("valami: %f", msg->twist.twist.linear.x);

        }

        lastOdometry_rear = msg;
    }

    void time_to_collision(double distance, double relative_speed, double relative_accel)
    {
  
        if(accel_rear >= 0.0){
            ttc = distance/relative_speed;
        }
        
        else{
            ttc = (-relative_speed - sqrt(pow(relative_speed,2)- 2 * relative_accel * distance)) / relative_accel;
        }

    }

    void acc(double speed_rear, double speed_desired){
        
        if(ttc < ttc_min){
            speed_rear -= 0.5;

            if(speed_rear <= 0.0){
                speed_rear = 0.0;
            }
        }
        
        if(speed_rear <= speed_desired){
            while(ttc > ttc_max){
                speed_rear += 0.5;
            }
        }

        if(ttc_min < ttc && ttc < ttc_max){
            speed_rear = speed_rear;
        }

        if(distance == 33){
            speed_rear = speed_desired;
        }

        speed_rear_out.drive.speed = speed_rear;
        speedout_rear.publish(speed_rear_out);
        speed_front_out.drive.speed = speed_front;
        speedout_front.publish(speed_front_out);

    }

    double speed_front = 0.1;
    double speed_rear;
    double relative_speed;
    double relative_accel;
    double accel_front;
    double accel_rear;
    double distance;
    double ttc;
    double ttc_min = 3.0;
    double ttc_max = 5.0;
    double speed_desired = 0.2;
    nav_msgs::Odometry::ConstPtr lastOdometry_front;
    nav_msgs::Odometry::ConstPtr lastOdometry_rear;
    
    ackermann_msgs::AckermannDriveStamped speed_rear_out;
    ackermann_msgs::AckermannDriveStamped speed_front_out;


    ros::Publisher speedout_rear;
    ros::Publisher speedout_front;
    ros::Subscriber speedin_rear;
    ros::Subscriber scanin;
    ros::Subscriber speedin_front;
    ros::Subscriber odometry_sub_front;
    ros::Subscriber odometry_sub_rear;
    ros::NodeHandle n;
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"acc_node");
    ros::NodeHandle nh("~");
    acc_pub x(nh);
    ros::spin();
}