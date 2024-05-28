#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

def calculate_midpoint(x1, y1, x2, y2):
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2
    return mx, my

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def calculate_unit_vector(x1, y1, x2, y2, distance):
    ux = (x2 - x1) / distance
    uy = (y2 - y1) / distance
    return ux, uy

def find_perpendicular_points_near_midpoint(x1, y1, x2, y2, distance_from_midpoint):
    mx, my = calculate_midpoint(x1, y1, x2, y2)
    distance = calculate_distance(x1, y1, x2, y2)
    ux, uy = calculate_unit_vector(x1, y1, x2, y2, distance)
    
    # 垂直向量
    vx1, vy1 = -uy, ux
    vx2, vy2 = uy, -ux
    
    point1 = (mx + distance_from_midpoint * vx1, my + distance_from_midpoint * vy1)
    point2 = (mx + distance_from_midpoint * vx2, my + distance_from_midpoint * vy2)
    
    return point1, point2
"""
# 示例使用
x1, y1 = 1, 2
x2, y2 = 4, 6
distance_from_midpoint = 0.2

point1, point2 = find_perpendicular_points_near_midpoint(x1, y1, x2, y2, distance_from_midpoint)
print(f"Point 1: {point1}")
print(f"Point 2: {point2}")
"""


class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('laser_scan_processor', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_pose_pub = rospy.Publisher('/target_pose', Pose2D, queue_size=10)
        self.target_detected = False
        
        

    def scan_callback(self, data):
        far_intensity = -1.0  #用于存储最高反射强度值，初始值设为-1.0
        far_index = -1  #用于存储最高反射强度值的索引，初始值设为-1
        value_far=10
        
        close_intensity=-1.0
        close_index=-1
        value_close=10
        
        for i in range(len(data.ranges)):
            
            if data.intensities[i] > value_close and close_index==-1:
                close_intensity = data.intensities[i]
                close_index = i
                
            if data.intensities[i] > value_far :
                far_intensity = data.intensities[i]
                far_index = i
        
        
        
        if far_index != -1 and close_index != -1:
        
            angle_far = data.angle_min + far_index * data.angle_increment
            angle_close=data.angle_min + close_index * data.angle_increment
            
            distance_far = data.ranges[far_index]
            distance_close = data.ranges[close_index]
            rospy.loginfo(f"angle={angle_far}, distance={distance_far},{far_index}")
            rospy.loginfo(f"angle={angle_close}, distance={distance_close},{close_index}")
            
            far_x=distance_far * math.cos(angle_far)
            far_y=distance_far * math.sin(angle_far)
            
            close_x=distance_close * math.cos(angle_close)
            close_y=distance_close * math.sin(angle_close)
            
            p1,p2=find_perpendicular_points_near_midpoint(close_x,close_y,far_x,far_y,0.2)
            
            
            target_pose = Pose2D()
            
            if (p2[0]**2+p2[1]**2)<(p1[0]**2+p1[1]**2):
            	target_pose.x=p2[0]
            	target_pose.y=p2[1]
            else:
            	target_pose.x=p1[0]
            	target_pose.y=p1[1]
            
            
            mx, my = calculate_midpoint(close_x,close_y,far_x,far_y,)
            angle = calculate_angle(mx, my, target_pose.x,target_pose.y)
            target_pose.theta = angle/180*3.14+3.14

            rospy.loginfo(f"Target Pose Received: x={target_pose.x}, y={target_pose.y}, theta={target_pose.theta}")
            self.target_pose_pub.publish(target_pose)
            self.target_detected = True

if __name__ == '__main__':
    try:
        processor = LaserScanProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

