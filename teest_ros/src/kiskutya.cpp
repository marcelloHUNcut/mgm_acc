#include <vector>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "ros/ros.h"

nav_msgs::Odometry last_msg;
ros::Publisher pub;
std_msgs::Float64 sum_elmozdulas;
ros::Publisher pub_odom;


void callback_odom(nav_msgs::Odometry msg)
{
    
    double ds=sqrt(pow(msg.pose.pose.position.x-last_msg.pose.pose.position.x,2)+pow(msg.pose.pose.position.y-last_msg.pose.pose.position.y,2));
    last_msg=msg;

    sum_elmozdulas.data+=ds;

    pub.publish(sum_elmozdulas);
    int last_sum_elmozdulas=sum_elmozdulas.data; //egésszé alakítás
    int actual_elmozdulas=sum_elmozdulas.data;
    if (last_sum_elmozdulas!=actual_elmozdulas)
    {
        pub_odom.publish(msg);


    }
    
}

int main(int argc, char** argv)
{
    std::vector<double> a;
    a.push_back(10.0);
    a.push_back(10.0);
    a.push_back(10.0);
    a.push_back(2.0);
    a.push_back(10.0);
    a.push_back(10.0);
    a.push_back(10.0);
    a.push_back(10.0);

    //std::size_t x= a.size();

    int p=a.size();

    a.size();
    a.empty();
    
    std::vector<nav_msgs::Odometry> odooom;
    std::map<std::string, int> varosok;

    varosok["Budapest"]=1000000;
    varosok["Kecskemét"]=10000;


    auto odooom1=odooom;


    ros::init (argc, argv, "zh_node1");
    ros:: NodeHandle nh("~");
    ros::Subscriber sub=nh.subscribe("/agent1/odom/ground_truth",1,callback_odom);
    ros::Publisher pub=nh.advertise<std_msgs::Float64>("/elmozdulas",1);
    ros::Publisher pub_odom=nh.advertise<nav_msgs::Odometry>("/poz",1);
    std_msgs::Float64 xx;
    pub.publish(xx);






   //várakoztatás
    ros::spin();
    ros::Rate x=ros::Rate(10); //hertzben
    while (ros::ok())
    {
        //kóóód


        ros::spinOnce();
        x.sleep();
    }
    



    return 0;
}