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
        angle_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/agent1/ackermann_cmd",1);

    } 
    
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        distance_min = 33;
        double distance;
        speed_control_front(speed_front_in);
        angle_control(angle);
        relative_speed = speed_front_in - speed_rear_in;

        int j = -1;
        for(int i=0; i<181;i++){
                     
            if((scan_in->range_min < scan_in->ranges[j]) && (scan_in->ranges[j] < scan_in->range_max)){

                distance = scan_in->ranges[j];
                

                if(distance < distance_min){
                    distance_min = distance;
                    
                }
                
            }
                    
            if(j==90){
                j = 270;
            }
            j++;

        }
        ROS_INFO("distance %f", distance_min);

        time_to_collision(distance_min, relative_speed);
        acc(speed_rear_in, speed_desired, distance_min, relative_speed);

        ROS_INFO("ttc: %f", ttc);
        //ROS_INFO("distance_min: %f", distance_min);
        //ROS_INFO("speed_rear_in: %f", speed_rear_in);
        //ROS_INFO("speed_front_in: %f", speed_front_in);
        ROS_INFO("relative_speed %f", relative_speed);

    }

    void scan_callback_front(const nav_msgs::Odometry &odom_front){

        speed_front_in = odom_front.twist.twist.linear.x;
        if(speed_front_in < 0){
            speed_front_in = 0;
        }

    }
    
    void scan_callback_rear(const nav_msgs::Odometry &odom_rear){

        speed_rear_in = odom_rear.twist.twist.linear.x;
        if(speed_rear_in < 0){
            speed_rear_in = 0;
        }
        
    }

    void time_to_collision(double distance_min, double relative_speed)
    {
  
        ttc = -distance_min/relative_speed;

    }

    void acc(double speed_rear_in, double speed_desired, double distance_min, double relative_speed){
        
        if((0 < ttc && ttc < ttc_min && relative_speed < 0) || distance_min < 0.6){
            speed_rear_out = speed_rear_in - 0.007;
            ROS_INFO("anyad");
        }

        if(distance_min < 0.6 && speed_rear_in < 0.1){
            speed_rear_out = speed_rear_in - 0.01;
        }
        
        if((((ttc < 0) || (ttc > ttc_max+0.2)) && distance_min > 0.8) && speed_rear_out < speed_desired){
            speed_rear_out = speed_rear_in + 0.007;
        }

        if(distance_min == 33 || relative_speed > 0.1 && speed_rear_out < speed_desired){
            speed_rear_out = speed_rear_in + 0.01;
        }

        if(speed_rear_out >= speed_desired){
            speed_rear_out = speed_desired;
        }
        
        if(speed_rear_in <= 0.001){
            speed_rear_out = 0.0;

        }
        if(ttc < 0 && relative_speed > 0.25 && speed_rear_out < speed_desired){
            speed_rear_out = speed_rear_in + 0.01;
        }

        if(distance_min < 0.3){
            speed_rear_out = speed_rear_in - 0.13;
            ROS_INFO("COLLISION WARNING!");
        }
    
        ROS_INFO("speed_rear_out %f", speed_rear_out);
        speed_rear_out_ack.header.stamp = ros::Time::now();
        speed_rear_out_ack.drive.speed = speed_rear_out;
        speedout_rear.publish(speed_rear_out_ack);

    }
    

    void speed_control_front(double speed_front_in){
        ros::Time currentTime = ros::Time::now();
        time = currentTime.toSec();

        if(time > 0.0 && time < 20){
            speed_front_out = 0.3;
        }
        if(time > 20 && time < 25){
            speed_front_out = 0.2;
        }
        if(time > 23.0 && time < 26.0){
            speed_front_out = speed_front_in - 0.01;
        }
        if(time > 28.0 && time < 31){
            speed_front_out = speed_front_in + 0.01;
        }
        if(time > 31.0 && time < 40.0){
            speed_front_out = 0.3;
        }
        if(time > 40){
            speed_front_out = 0.4;
        }
        /*if(time > 35){
            speed_front_out = speed_front_in - 0.01;
        }*/
        if(time > 53){
            speed_front_out = 0;
        }
        if(speed_front_in <= 0.0 && 10 < time && time < 28 ){
            speed_front_out = 0.0;
        }

        //ROS_INFO("speed_front_out %f", speed_front_out);

        speed_front_out_ack.header.stamp = ros::Time::now();
        speed_front_out_ack.drive.speed = speed_front_out;
        speedout_front.publish(speed_front_out_ack);


    }
    void angle_control(float angle){
        ros::Time currentTime = ros::Time::now();
        time = currentTime.toSec();
        if(time > 45 && time < 47){
            angle_ack.drive.steering_angle = angle;
            angle_pub.publish(angle_ack);
        }
    }

    double speed_front_in;
    double speed_front_out;
    double speed_rear_out;
    double speed_rear_in;
    double relative_speed;
    double relative_accel;
    double accel_front;
    double accel_rear;
    double distance_min;
    double ttc;
    double ttc_min = 5.0;
    double ttc_max = 7.0;
    double speed_desired = 0.5;
    double time;
    float angle = -5.0;
    
    ackermann_msgs::AckermannDriveStamped speed_rear_out_ack;
    ackermann_msgs::AckermannDriveStamped speed_front_out_ack;
    ackermann_msgs::AckermannDriveStamped angle_ack;


    ros::Publisher speedout_rear;
    ros::Publisher speedout_front;
    ros::Publisher angle_pub;
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