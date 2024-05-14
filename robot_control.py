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
        
        
        self.pose_sub = rospy.Subscriber("estimation", PoseWithCovarianceStamped, self.pose_callback)

        self.current_pose = None  # 初始化存储当前位姿的变量
        # self.current_pose = [0, 0, 0]  # x, y, yaw/theta
        self.current_scan = None
        self.goal = {'x': 0.0, 'y': -10.0}  # 目标位置

    def pose_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_euler(msg.pose.pose.orientation)
        }
        #print('X : ',msg.pose.pose.position.x)
        #print('Y : ',msg.pose.pose.position.y)


    def quaternion_to_euler(self, quat):
        # 此函数将四元数转换为欧拉角，假设四元数是标准化的
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
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.current_scan and self.current_pose:
                command = Twist()
                # 确定避障策略
                command = self.calculate_avoidance(self.current_scan, self.current_pose)
                self.vel_pub.publish(command)
            else:
                rospy.logwarn("No laser scan or pose data available.")
            rate.sleep()

########## 旋转方向，前进角度待修改，证实
#  坐标系的方向是什么??????
    def calculate_avoidance(self, scan, pose):
        command = Twist()
        attract = self.calculate_attractive_force(pose)
        repulse = self.calculate_repulsive_force(scan, pose)

        # 计算总力
        #force_x = attract['x'] + repulse['x']
        #force_y = attract['y'] + repulse['y']
        force_x = attract['x'] 
        force_y = attract['y']
        #print('force_x :' ,force_x)
        #print('force_y :' ,force_y)
        # 目标角度（弧度）
        target_angle = math.atan2(force_y, force_x)
    
        # 当前朝向
        current_angle = pose['theta']
    
        # 角度差
        angle_diff = self.normalize_angle(target_angle - current_angle)

        # 将力转换为速度指令,avec questions
        
        command.angular.z = 2 * angle_diff  # 可以调整0.3这个系数来改变旋转的快慢  逆时针转动是用正值
        command.linear.x = 0.1 * math.sqrt(force_x**2 + force_y**2)
        print('command.angular.z :' ,command.angular.z)
        print('command.linear.x :' ,command.linear.x)
        
        if command.linear.x < 0.1:
                command.linear.x = 0
                if angle_diff < 0.001:
                        command.angular.z = 0
	
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
                return {'x': force_x / norm, 'y': force_y / norm}
                #return {'x': force_x , 'y': force_y }
        else:
                return {'x': 0, 'y': 0}

    def calculate_repulsive_force(self, scan, pose):
        # 斥力由附近的障碍物产生
        force_x = 0
        force_y = 0
        for i, distance in enumerate(scan):
            if distance < 1:  # 斥力作用距离
                # 计算障碍物方向
                angle = i * math.pi / len(scan) - math.pi / 2
                
                #angle = (i + 1) * math.pi / len(scan)
                
                # 计算斥力分量
                force_x -= (1 - distance) * math.cos(angle)
                force_y -= (1 - distance) * math.sin(angle)
                
                #force_x -= (0.5 - distance) * math.sin(angle)
                #force_y -= (0.5 - distance) * math.cos(angle)

        return {'x': force_x, 'y': force_y}


if __name__ == '__main__':
    controller = RobotController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass

