#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


import tf2_ros
import tf2_geometry_msgs


class scan_check:
    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pub_pose = rospy.Publisher("/pose_out",PoseStamped, queue_size=1)
        self.pub_dist = rospy.Publisher("/distance",Float64, queue_size=1)
        self.sub = rospy.Subscriber("/agent1/scan", LaserScan, self.scan_callback, queue_size=1)



    def scan_callback(self, msg: LaserScan):


        closest_dist = 1000
        closest_dist_angel = 0

        for i in range(len(msg.ranges)):
            if(closest_dist > msg.ranges[i]):
                closest_dist = msg.ranges[i]
                closest_dist_angel = i

        angle = msg.angle_min + closest_dist_angel*msg.angle_increment 
        
        closest_point = PoseStamped()
        closest_point.header = msg.header
        closest_point.pose.position.x = closest_dist*math.cos(angle)
        closest_point.pose.position.y = closest_dist*math.sin(angle)
        closest_point.pose.position.z = 0

        # A quaternion nem lehet (0,0,0,0) értékű
        closest_point.pose.orientation.w = 1

        closest_point_map = PoseStamped()
        if self.tfBuffer.can_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp)

            closest_point_map = tf2_geometry_msgs.do_transform_pose(closest_point, trans_base2map)

        self.pub_pose.publish(closest_point_map)

        distance = Float64()
        distance.data = closest_dist
        self.pub_dist.publish(distance)
        


if __name__ == '__main__':
    rospy.init_node("scan_checker")

    scan_checker = scan_check()
    
    rospy.spin()
