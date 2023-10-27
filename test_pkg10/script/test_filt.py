#!/usr/bin/env python3

import math
import copy
import rospy

from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import *



class filter:
    def __init__(self):

        self.avg_pub = rospy.Publisher("/avg_dist",Float64, queue_size=1)
        self.pose_pub = rospy.Publisher("/mid_point",PoseStamped, queue_size=1)

        self.path_sub = rospy.Subscriber("/high_dist", Path, self.path_callback, queue_size=1)
        self.dist_sub = rospy.Subscriber("/distances", Float64MultiArray, self.dist_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        self.avg_distances = []

    def dist_callback(self, msg: Float64MultiArray):
        avg_dist = sum(msg.data)/len(msg.data)
        self.avg_distances.append(avg_dist)
        if (len(self.avg_distances) > 10):
            self.avg_distances.pop(0)

    def path_callback(self, msg: Path):
        mid_point = PoseStamped()
        mid_point.header = msg.header
        mid_point.pose.position.x = (msg.poses[0].pose.position.x + msg.poses[1].pose.position.x)/2
        mid_point.pose.position.y = (msg.poses[0].pose.position.y + msg.poses[1].pose.position.y)/2
        mid_point.pose.position.z = (msg.poses[0].pose.position.z + msg.poses[1].pose.position.z)/2

        dx = msg.poses[0].pose.position.x - msg.poses[1].pose.position.x
        dy = msg.poses[0].pose.position.y - msg.poses[1].pose.position.y

        angle = math.atan2(dy,dx) + math.pi/2

        q = quaternion_from_euler(0,0,angle)
        mid_point.pose.orientation.x = q[0]
        mid_point.pose.orientation.y = q[1]
        mid_point.pose.orientation.z = q[2]
        mid_point.pose.orientation.w = q[3]

        self.pose_pub.publish(mid_point)

    def timer_callback(self, event):
        if (len(self.avg_distances) >0):
            avg_dist = Float64()
            avg_dist.data = sum(self.avg_distances)/len(self.avg_distances)
            self.avg_pub.publish(avg_dist)

        
if __name__ == '__main__':
    rospy.init_node("filter")

    filtering = filter()
    
    rospy.spin()
