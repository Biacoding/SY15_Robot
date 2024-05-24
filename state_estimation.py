#!/usr/bin/python3

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class StateEstimation:
    def __init__(self):
        rospy.init_node("state_estimation", log_level=rospy.INFO)
        rospy.loginfo("Démarrage du nœud d'estimation d'état")
        
        self.estimate_publisher = rospy.Publisher("estimation", PoseWithCovarianceStamped, queue_size=1)
        
        self.command_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.receive_input)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.receive_imu)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.X = np.array([[self.x], [self.y], [self.theta], [self.v], [self.w]])
        self.deltaT = 1 / 30
        self.P = np.eye(5) * 0.01
        self.F = np.eye(5)
        self.Q = np.array([[0.01, 0, 0, 0, 0],
                           [0, 0.1, 0, 0, 0],
                           [0, 0, 0.01, 0, 0],
                           [0, 0, 0, 0.01, 0],
                           [0, 0, 0, 0, 0.01]])
        self.R = np.array([[0.01, 0],
                            [0, 0.1]])
        self.K = np.zeros((5, 2))
        self.Z = np.array([[self.v], [self.w]])

        self.C = np.array([[0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])

        self.timer = rospy.Timer(rospy.Duration(self.deltaT), self.timer_callback)

    def timer_callback(self, event):
        # Prediction step
        self.x += self.v * math.cos(self.theta) * self.deltaT
        self.y += self.v * math.sin(self.theta) * self.deltaT
        self.theta += self.w * self.deltaT
        self.v = self.X[3][0]
        self.w = self.X[4][0]

        self.X = np.array([[self.x], [self.y], [self.theta], [self.v], [self.w]])

        self.F[0][2] = -self.v * self.deltaT * math.sin(self.theta)
        self.F[0][3] = self.deltaT * math.cos(self.theta)
        self.F[1][2] = self.v * self.deltaT * math.cos(self.theta)
        self.F[1][3] = self.deltaT * math.sin(self.theta)
        self.F[2][4] = self.deltaT

        self.P = self.F @ self.P @ self.F.T + self.Q

        # Correction step
        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        self.X = self.X + self.K @ (self.Z - self.C @ self.X)
        self.P = (np.eye(5) - self.K @ self.C) @ self.P

        self.publish_estimate()

    def receive_input(self, twist_msg: Twist):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z

        # self.v = linear_velocity
        # self.w = angular_velocity

    def receive_odom(self, odom_msg: Odometry):
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z

        self.Z[0][0] = linear_velocity
        self.Z[1][0] = angular_velocity

    def receive_imu(self, imu_msg: Imu):
        linear_acceleration = imu_msg.linear_acceleration # Attention, contient .x .y et .z
        angular_velocity = imu_msg.angular_velocity.z

    def publish_estimate(self):
        x = self.X[0][0]
        y = self.X[1][0]
        theta = self.X[2][0]


        covariance = self.P

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)

        msg.pose.covariance[0] = covariance[0][0]
        msg.pose.covariance[1] = covariance[0][1]
        msg.pose.covariance[5] = covariance[0][2]
        msg.pose.covariance[6] = covariance[1][0]
        msg.pose.covariance[7] = covariance[1][1]
        msg.pose.covariance[11] = covariance[1][2]
        msg.pose.covariance[30] = covariance[2][0]
        msg.pose.covariance[31] = covariance[2][1]
        msg.pose.covariance[35] = covariance[2][2]

        self.estimate_publisher.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = StateEstimation()
    node.run()
