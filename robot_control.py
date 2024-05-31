#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import numpy as np
import math
import tf.transformations

class RobotController:
    def __init__(self):
        rospy.init_node('robot_control', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pose_sub = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pose_callback)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)

        self.current_pose = None  # 初始化存储当前位姿的变量
        self.current_scan = None
        self.goal = {'x': -1.0, 'y': 1.0}  # 目标位置

    def pose_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_euler(msg.pose.pose.orientation)
        }

    def quaternion_to_euler(self, quat):
        # 将四元数转换为欧拉角，假设四元数是标准化的
        euler = tf.transformations.euler_from_quaternion([
            quat.x,
            quat.y,
            quat.z,
            quat.w
        ])
        return euler[2]  # 返回偏航角

    def laser_callback(self, data):
        self.current_scan = data.ranges

    def run(self):
        rate = rospy.Rate(10)  # 50Hz
        while not rospy.is_shutdown():
            if self.current_scan and self.current_pose:
                command = self.calculate_avoidance(self.current_scan, self.current_pose)
                self.vel_pub.publish(command)
            else:
                rospy.logwarn("No laser scan or pose data available.")
            rate.sleep()

    def calculate_avoidance(self, scan, pose):
        command = Twist()
        attract = self.calculate_attractive_force(pose)
        #repulse = self.calculate_repulsive_force(scan, pose)

        # 计算总力
        #force_x = attract['x'] + repulse['x']
        #force_y = attract['y'] + repulse['y']
        force_x = attract['x']
        force_y = attract['y']
        # 目标角度（弧度）
        target_angle = math.atan2(force_y, force_x)
    
        # 当前朝向
        current_angle = pose['theta']
        print(math.cos(current_angle))
        # 角度差
        angle_diff = self.normalize_angle(target_angle - current_angle)
        
        # 将力转换为速度指令
        dist = math.sqrt(force_x**2 + force_y**2)
        if abs(angle_diff) > 0.05 and dist > 0.05:
              command.angular.z = 0.5 * angle_diff/abs(angle_diff)  # 调整旋转速度系数
        else:
              if dist > 0.05:
                  command.linear.x = 0.3    # 调整前进速度系数
        print(command.angular.x, command.angular.z)
        return command

    def normalize_angle(self, angle):
        # 规范化角度到 [-π, π]区间
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_attractive_force(self, pose):
        # 吸引力向目标方向
        force_x = self.goal['x'] - pose['x']
        force_y = self.goal['y'] - pose['y']
        # 简单归一化
        norm = math.sqrt(force_x**2 + force_y**2)
        if norm > 0:
            return {'x': force_x, 'y': force_y}
            #return {'x': force_x / norm, 'y': force_y / norm}
        else:
            return {'x': 0, 'y': 0}

    def calculate_repulsive_force(self, scan, pose):
        # 斥力由附近的障碍物产生
        force_x = 0
        force_y = 0
        for i, distance in enumerate(scan):
            if distance < 1.0:  # 斥力作用距离
                angle = i * (2 * math.pi / len(scan)) - math.pi  # 计算障碍物方向
                force_x -= (1.0 - distance) * math.cos(angle)
                force_y -= (1.0 - distance) * math.sin(angle)
        return {'x': force_x, 'y': force_y}


if __name__ == '__main__':
    controller = RobotController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
