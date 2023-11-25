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
       /*odometry_sub_front = n.subscribe("/agent1/odom/ground_truth", 1, &acc_pub::odometryCallback_front, this);
       odometry_sub_rear = n.subscribe("/agent2/odom/ground_truth", 1, &acc_pub::odometryCallback_rear, this);*/

    } 
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {

        speed_control_front();
        relative_speed = speed_front_in - speed_rear_in;
        //relative_accel = accel_front - accel_rear;
       
        if((scan_in->range_min < scan_in->ranges[0]) && (scan_in->ranges[0] < scan_in->range_max)){

            // TODO: Filter
            distance = scan_in->ranges[0];
        
        }
        else{
            distance = 33;
        }


        time_to_collision(distance, relative_speed);
        acc(speed_rear_in, speed_desired);

        ROS_INFO("ttc: %f", ttc);
        ROS_INFO("distance: %f", distance);
        ROS_INFO("speed_rear: %f", speed_rear_in);
        //ROS_INFO("accel_rear: %f", accel_rear);
        ROS_INFO("speed_front_in: %f", speed_front_in);
        //ROS_INFO("accel_front: %f", accel_front);
        ROS_INFO("relative_speed %f", relative_speed);
        //ROS_INFO("relative_accel %f", relative_accel);

    }

    void scan_callback_front(const nav_msgs::Odometry &odom_front){

        speed_front_in = odom_front.twist.twist.linear.x;

    }
    
    void scan_callback_rear(const nav_msgs::Odometry &odom_rear){

        speed_rear_in = odom_rear.twist.twist.linear.x;
        
    }

    /*void odometryCallback_front(const nav_msgs::Odometry::ConstPtr& msg) {
        if (lastOdometry_front != nullptr) {
            ros::Time currentTime = ros::Time::now();
            double deltaTime = (currentTime - lastOdometry_front->header.stamp).toSec();

            double linearVelocity = lastOdometry_front->twist.twist.linear.x;
            double test_front = msg->twist.twist.linear.x - linearVelocity;
            accel_front = test_front / deltaTime;
            //ROS_INFO("deltaTime_front: %f", deltaTime);
            //ROS_INFO("linearVelocity_front: %f", linearVelocity);
            //ROS_INFO("test_front %f", test_front);
        }

        lastOdometry_front = msg;
    }*/

    /*void odometryCallback_rear(const nav_msgs::Odometry::ConstPtr& msg) {
        if (lastOdometry_rear != nullptr) {
            ros::Time currentTime = ros::Time::now();
            double deltaTime = (currentTime - lastOdometry_rear->header.stamp).toSec();

            double linearVelocity = lastOdometry_rear->twist.twist.linear.x;
            double test_rear = msg->twist.twist.linear.x - linearVelocity;
            accel_rear = test_rear / deltaTime;
            //ROS_INFO("deltaTime_rear: %f", deltaTime);
            //ROS_INFO("linearVelocity_rear: %f", linearVelocity);
            //ROS_INFO("test_rear: %f", test_rear);
            //ROS_INFO("valami: %f", msg->twist.twist.linear.x);

        }

        lastOdometry_rear = msg;
    }*/

    void time_to_collision(double distance, double relative_speed)
    {
  
        if(relative_speed < 0){
            ttc = -distance/relative_speed;
        }
        if(relative_speed >= 0 || distance == 33){
            ttc = -3;
        }

            /*if(relative_speed != 0){
                ttc = 3;
            }
            else{
                ttc = 10;
            }
        }
        
        else{
            ttc = 20;//(-relative_speed - sqrt(pow(relative_speed,2)- 2 * relative_accel * distance)) / relative_accel;
        }*/

    }

    void acc(double speed_rear_in, double speed_desired){
        
        if(ttc < ttc_min){
            speed_rear_out = speed_rear_in - 0.1;

            if(speed_rear_in <= 0.0){
                speed_rear_out = 0.0;
            }
        }
        
        if(ttc == -3){
            speed_rear_out = speed_rear_in + 0.1;
        }

        if(speed_rear_out >= speed_desired){
            speed_rear_out = speed_desired;
        }
  

        speed_rear_out_ack.drive.speed = speed_rear_out;
        speedout_rear.publish(speed_rear_out_ack);

    }

    void speed_control_front(){
        ros::Time currentTime = ros::Time::now();
        time = currentTime.toSec();
        //ROS_INFO("time: %f", time);

        if(time > 5.0 && time < 10.0){
            speed_front_out = 0.1;
        }
        if(time > 10.0 && time < 15.0){
            speed_front_out = 0.5;
        }
        if(time > 15.0){
            speed_front_out= 0.3;
        }

        //ROS_INFO("speed_front_out %f", speed_front_out);

        speed_front_out_ack.drive.speed = speed_front_out;
        speedout_front.publish(speed_front_out_ack);
    }

    double speed_front_in;
    double speed_front_out;
    double speed_rear_out;
    double speed_rear_in;
    double relative_speed;
    double relative_accel;
    double accel_front;
    double accel_rear;
    double distance;
    double ttc;
    double ttc_min = 50.0;
    double speed_desired = 0.7;
    double time;
    nav_msgs::Odometry::ConstPtr lastOdometry_front;
    nav_msgs::Odometry::ConstPtr lastOdometry_rear;
    
    ackermann_msgs::AckermannDriveStamped speed_rear_out_ack;
    ackermann_msgs::AckermannDriveStamped speed_front_out_ack;


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