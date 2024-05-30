#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class RobotController:
    def __init__(self):
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()
        self.target_received = False
        rospy.init_node('robot_controller', anonymous=True)
        
        self.current_pose_sub = rospy.Subscriber("estimation", PoseWithCovarianceStamped, self.current_pose_callback)
        self.target_pose_sub = rospy.Subscriber('/target_pose', Pose2D, self.target_pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def target_pose_callback(self, msg):
        self.target_pose.x = msg.x+self.current_pose.x
        self.target_pose.y = msg.y+self.current_pose.y
        self.target_pose.theta = msg.theta+self.current_pose.theta
        self.target_received = True
        rospy.loginfo(f"Target Pose Received: x={self.target_pose.x}, y={self.target_pose.y}, theta={self.target_pose.theta}")
        

    def current_pose_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # 假设姿态是通过四元数表示的，需要将其转换为欧拉角
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_pose.theta = yaw
        rospy.loginfo(f"Current Pose Received: x={self.current_pose.x}, y={self.current_pose.y}, theta={self.current_pose.theta}")

    def move_to_target(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.target_received:
                dx = self.target_pose.x - self.current_pose.x
                dy = self.target_pose.y - self.current_pose.y
                distance = math.sqrt(dx * dx + dy * dy)
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.current_pose.theta

                cmd_vel = Twist()

                if distance > 0.02:  # 距离大于2厘米时移动
                    cmd_vel.linear.x = 0.5 * distance
                    cmd_vel.angular.z = 4.0 * angle_diff
                else:  # 调整朝向
                    cmd_vel.angular.z = 4.0 * angle_diff
                    if abs(angle_diff) < 0.1:
                        self.target_received = False  # 到达目标位置并朝向标志牌后停止

                self.cmd_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_to_target()
    except rospy.ROSInterruptException:
        pass


