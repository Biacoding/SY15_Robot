#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose2D, Twist

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.target_pose_sub = rospy.Subscriber('/target_pose', Pose2D, self.target_pose_callback)
        self.current_pose_sub = rospy.Subscriber("estimation", PoseWithCovarianceStamped, self.current_pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()
        self.target_received = False

    def target_pose_callback(self, msg):
        self.target_pose = msg
        self.target_received = True

    def current_pose_callback(self, msg):
        
        current_pose.x = msg.pose.pose.position.x
        current_pose.y = msg.pose.pose.position.y
        # 假设姿态是通过四元数表示的，需要将其转换为欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
        current_pose.theta = yaw
        
    def move_to_target(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.target_received:
                dx = self.target_pose.x - self.current_pose.x
                dy = self.target_pose.y - self.current_pose.y
                distance = math.sqrt(dx*dx + dy*dy)
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.current_pose.theta

                cmd_vel = Twist()

                if distance > 0.2:  # 距离大于20厘米时移动
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

