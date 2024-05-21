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
        
        #self.command_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.receive_input)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)
        #self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.receive_imu)
        
        self.X = np.zeros((5, 1))
        #self.X[0][0] = self.x
        #self.X[1][0] = self.y
        #self.X[2][0] = self.theta
        #self.X[3][0] = self.v
        #self.X[4][0] = self.w
        
        self.frenT = 1 / 10 # 20hz
        self.P = np.eye(5) * 0.01
        self.F = np.eye(5)
        #self.Q = np.eye(5) * 0.0001
        self.Q = np.diag([0.0001, 0.0001, 0.001, 0, 0])  # 调整过程噪声矩阵
        #self.R = np.eye(2) * 0.0001  # old
        self.R = np.array([[0.001, 0],	#0.001
                          [0, 0.00001]]) # 0.0001
        self.C = np.array([[0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1]])
        self.K = np.zeros((5, 2))
        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R) # Matrice de gain de Kalman
        #self.Z = np.array([[self.v], [self.w]]) # avec problems
        self.Z = np.zeros((2, 1))
                        
        self.last_time = rospy.Time.now().to_sec()
        self.now_time = rospy.Time.now().to_sec()
        self.deltaT = 0
        self.timer = rospy.Timer(rospy.Duration(self.frenT), self.timer_callback)

    def timer_callback(self, event):
        self.now_time = rospy.Time.now().to_sec()
        self.deltaT = self.now_time - self.last_time
    	
    	# Fk est la Jacobienne de f suivant X, au point Xk|k
        self.F[0][2] = -self.X[3][0] * self.deltaT * math.sin(self.X[2][0])
        self.F[0][3] = self.deltaT * math.cos(self.X[2][0])
        self.F[1][2] = self.X[3][0] * self.deltaT * math.cos(self.X[2][0])
        self.F[1][3] = self.deltaT * math.sin(self.X[2][0])
        self.F[2][4] = self.deltaT
    
        # Prediction step Xk+1/k = f(Xk/k)
        self.X[0][0] += self.X[3][0] * math.cos(self.X[2][0]) * self.deltaT
        self.X[1][0] += self.X[3][0] * math.sin(self.X[2][0]) * self.deltaT
        self.X[2][0] += self.X[4][0] * self.deltaT
        
  
	# Adjust Q based on current velocities
        self.Q[3][3] = self.X[3][0] * 0.05  
        self.Q[4][4] = self.X[4][0] * 0.05  
	
        #Pk+1|k = Fk Pk|k FTk + Q
        self.P = self.F @ self.P @ self.F.T + self.Q  # Pk+1/k
        
        self.last_time = self.now_time
        self.publish_estimate()
    
    # no use
    def receive_input(self, twist_msg: Twist):
        linear_velocity_input = twist_msg.linear.x
        angular_velocity_input = twist_msg.angular.z
        if abs(angular_velocity_input) > 2 :
            return
        self.X[3][0] = linear_velocity_input
        self.X[4][0] = angular_velocity_input

    def receive_odom(self, odom_msg: Odometry):
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z
        self.Z[0][0] = linear_velocity
        self.Z[1][0] = angular_velocity
        # Correction step
        self.X = self.X + self.K @ (self.Z - self.C @ self.X)
        self.P = (np.eye(5) - self.K @ self.C) @ self.P
        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
    # no use
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
