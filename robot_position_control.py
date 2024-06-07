#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class RobotController:
    def __init__(self):
        self.current_pose = Pose2D()
        self.target_pose = Pose2D()
        self.target_received = False

        self.laser_min = 0.0
        self.laser_ranges = []
        
        rospy.init_node('robot_controller', anonymous=True)
        
        self.scan = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.current_pose_sub = rospy.Subscriber("estimation", PoseWithCovarianceStamped, self.current_pose_callback)
        self.target_pose_sub = rospy.Subscriber('/target_pose', Pose2D, self.target_pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def laser_callback(self, msg):
        self.laser_min = msg.range_min
        self.laser_ranges = msg.ranges

    def target_pose_callback(self, msg):
        self.target_pose.x = msg.x
        self.target_pose.y = msg.y
        self.target_pose.theta = msg.theta
        self.target_received = True
        rospy.loginfo(f"Target Pose Received: x={self.target_pose.x}, y={self.target_pose.y}, theta={self.target_pose.theta}")
        

    def current_pose_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_pose.theta = yaw

    def move_to_target(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():

            if self.target_received and self.laser_ranges: # and self.laser_ranges and front_distance > 0.1:  
                circle_distance = min(self.laser_ranges)
                front_distance = min(min(self.laser_ranges[0:7]),  min(self.laser_ranges[-7:]))
                dx = self.target_pose.x - self.current_pose.x
                dy = self.target_pose.y - self.current_pose.y
                distance = math.sqrt(dx * dx + dy * dy)
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.current_pose.theta

                cmd_vel = Twist()

                if distance > 0.1:
                    cmd_vel.linear.x = 0.2
                    cmd_vel.angular.z = 0.5 * angle_diff
                elif front_distance > circle_distance:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.5
                else:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Reached the goal!")
                    break

                self.cmd_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.move_to_target()
    except rospy.ROSInterruptException:
        pass


