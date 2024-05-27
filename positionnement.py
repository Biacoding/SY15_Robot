#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('laser_scan_processor', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_pose_pub = rospy.Publisher('/target_pose', Pose2D, queue_size=10)
        self.target_detected = False

    def scan_callback(self, data):
        max_intensity = -1.0  #用于存储最高反射强度值，初始值设为-1.0
        max_index = -1  #用于存储最高反射强度值的索引，初始值设为-1
        for i in range(len(data.ranges)):
            if data.intensities[i] > max_intensity:
                max_intensity = data.intensities[i]
                max_index = i

        if max_index != -1:
            angle = data.angle_min + max_index * data.angle_increment
            distance = data.ranges[max_index]
            rospy.loginfo(f"angle={angle}, distance={distance},{max_index}")
            target_pose = Pose2D()
            target_pose.x = distance * math.cos(angle)
            target_pose.y = distance * math.sin(angle)
            target_pose.theta = angle

            rospy.loginfo(f"Target Pose Received: x={target_pose.x}, y={target_pose.y}, theta={target_pose.theta}")
            self.target_pose_pub.publish(target_pose)
            self.target_detected = True

if __name__ == '__main__':
    try:
        processor = LaserScanProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

