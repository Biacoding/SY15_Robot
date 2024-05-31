#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

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

def calculate_angle(mx, my, px, py):
    angle_radians = math.atan2(py - my, px - mx)
    return angle_radians

class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('laser_scan_processor', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.target_pose_pub = rospy.Publisher('/target_pose', Pose2D, queue_size=10)
        self.current_pose_sub = rospy.Subscriber("estimation", PoseWithCovarianceStamped, self.current_pose_callback)
        self.target_detected = False
        self.current_pose = Pose2D()
        self.x_fixed = 0
        self.y_fixed = 0
        self.theta_fixed = 0
        self.once = False    
    
    def current_pose_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # 假设姿态是通过四元数表示的，需要将其转换为欧拉角
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_pose.theta = yaw
        # rospy.loginfo(f"Current Pose Received: x={self.current_pose.x}, y={self.current_pose.y}, theta={self.current_pose.theta}")

        
    def scan_callback(self, data):
        far_intensity = -1.0
        far_index = -1
        value_far = 0.5
        
        close_intensity = -1.0
        close_index = -1
        value_close = 0.5

        # 定义合理的距离范围
        min_distance = 0.1
        max_distance = 10.0
        
        for i in range(len(data.ranges)):
            if min_distance <= data.ranges[i] <= max_distance:
                if data.intensities[i] > value_close and (close_index == -1 or data.ranges[i] < data.ranges[close_index]):
                    close_intensity = data.intensities[i]
                    close_index = i

                if data.intensities[i] > value_far and (far_index == -1 or data.ranges[i] > data.ranges[far_index]):
                    far_intensity = data.intensities[i]
                    far_index = i

        if far_index != -1 and close_index != -1:
            angle_far = data.angle_min + far_index * data.angle_increment
            angle_close = data.angle_min + close_index * data.angle_increment
            
            distance_far = data.ranges[far_index]
            distance_close = data.ranges[close_index]
            rospy.loginfo(f"angle_far={angle_far}, distance_far={distance_far}, index_far={far_index}")
            rospy.loginfo(f"angle_close={angle_close}, distance_close={distance_close}, index_close={close_index}")
            
            far_x = distance_far * math.cos(angle_far)
            far_y = distance_far * math.sin(angle_far)
            
            close_x = distance_close * math.cos(angle_close)
            close_y = distance_close * math.sin(angle_close)
            
            p1, p2 = find_perpendicular_points_near_midpoint(close_x, close_y, far_x, far_y, 0.2)
            
            target_pose = Pose2D()
            
            if (p2[0]**2 + p2[1]**2) < (p1[0]**2 + p1[1]**2):
                target_pose.x = p2[0]
                target_pose.y = p2[1]
            else:
                target_pose.x = p1[0]
                target_pose.y = p1[1]
            
            mx, my = calculate_midpoint(close_x, close_y, far_x, far_y)
            angle = calculate_angle(mx, my, target_pose.x, target_pose.y)
            target_pose.theta = angle

            target_pose.x += self.current_pose.x
            target_pose.y += self.current_pose.y

            if not self.once:
                self.x_fixed = target_pose.x
                self.y_fixed = target_pose.y
                self.theta_fixed = target_pose.theta
                self.once = True
            once_pose = Pose2D(self.x_fixed, self.y_fixed, self.theta_fixed)
            
            rospy.loginfo(f"Target Pose: x={once_pose.x}, y={once_pose.y}, theta={once_pose.theta}")
            self.target_pose_pub.publish(once_pose)
            self.target_detected = True

if __name__ == '__main__':
    try:
        processor = LaserScanProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
