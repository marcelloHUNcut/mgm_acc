#!/usr/bin/env python3

import math
import copy
import rospy

from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import sensor_msgs.point_cloud2 as pcl2



class scan2pcl:
    def __init__(self):

        self.cloud_pub = rospy.Publisher("/cloud",PointCloud2, queue_size=1)
        self.dist_pub = rospy.Publisher("/distances",Float64MultiArray, queue_size=1)
        self.path_pub = rospy.Publisher("/high_dist",Path, queue_size=1)
        self.sub = rospy.Subscriber("/agent1/scan", LaserScan, self.scan_callback, queue_size=1)

        self.disances = Float64MultiArray()
        self.max_dist = 0.0

        self.path_out = Path()
        pose = PoseStamped()
        self.path_out.poses.append(pose)
        self.path_out.poses.append(copy.deepcopy(pose))

    def scan_callback(self, scan_in: LaserScan):

        self.max_dist = 0.0
        self.disances.data = []

        localCloud = []
        for i in range(len(scan_in.ranges)):
            if (scan_in.ranges[i] > scan_in.range_min and scan_in.ranges[i] < scan_in.range_max): 
                angle = scan_in.angle_min + i * scan_in.angle_increment
                x = scan_in.ranges[i] * math.cos(angle)
                y = scan_in.ranges[i] * math.sin(angle)
                z = 0

                if (len(localCloud) > 0):
                    self.calculate_distances(localCloud[-1], [x,y,z])

                localCloud.append([x,y,z])


        if(len(localCloud) > 2):
            self.calculate_distances(localCloud[-1], localCloud[0])

        temp = pcl2.create_cloud_xyz32(scan_in.header, localCloud)
        self.cloud_pub.publish(temp)

        self.dist_pub.publish(self.disances)

        self.path_out.header = scan_in.header
        self.path_pub.publish(self.path_out)



    def calculate_distances(self, point1, point2):
        dist = math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        self.disances.data.append(dist)

        if (dist > self.max_dist):
            self.max_dist = dist

            self.path_out.poses[0].pose.position.x = point1[0]
            self.path_out.poses[0].pose.position.y = point1[1]
            self.path_out.poses[0].pose.position.z = point1[2]

            self.path_out.poses[1].pose.position.x = point2[0]
            self.path_out.poses[1].pose.position.y = point2[1]
            self.path_out.poses[1].pose.position.z = point2[2]


        
if __name__ == '__main__':
    rospy.init_node("scan2pcl")

    scan_convetrter = scan2pcl()
    
    rospy.spin()
