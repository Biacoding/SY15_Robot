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
        rospy.Timer
        rospy.loginfo("Démarrage du nœud d'estimation d'état")
        
        self.estimate_publisher = rospy.Publisher("estimation", PoseWithCovarianceStamped, queue_size=1)
        
        self.command_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.receive_input)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.receive_imu)
        
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0 # 0.1
        self.w = 0
        self.X_etat = np.array([[self.x],[self.y],[self.theta],[self.v],[self.w]])
        
        self.deltaT = 1/20
        self.P = 0.00001*np.eye(5)  # Matrice de covariance de l'état
        
        self.F = np.eye(5)
        self.F[0][2] = -self.v * self.deltaT * math.sin(self.theta)
        self.F[0][3] = self.deltaT * math.cos(self.theta)
        self.F[1][2] = self.v*self.deltaT * math.cos(self.theta)
        self.F[1][3] = self.deltaT * math.sin(self.theta)
        self.F[2][4] = self.deltaT
        
        self.Q = np.eye(5) * 0.000001  # 5*5  Matrice de covariance du bruit de processus
        self.R = np.eye(2) # 2*2  Matrice de covariance du bruit de mesure
        self.R[0,0] = 0.001
        self.R[1,1] = 0.00001
        
        self.Z = np.array([[self.v],[self.w]])
        self.C = np.array([[0,0,0,1,0],[0,0,0,0,1]])
        
        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R) # 5*2 Matrice de gain de Kalman
        
        self.last_twist = Twist()  # initial default twist
        self.timer = rospy.Timer(rospy.Duration(1/20),self.update_state)

    def update_state(self, event):
        #rospy.loginfo("Updating Timer")
        self.receive_input(self.last_twist)
        
    # ÉTAPE DE PRÉDICTION
    def receive_input(self, twist_msg:Twist):
    	# receive command V W comme etat
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        self.v = linear_velocity
        self.w = angular_velocity
        
        # Calcul de la matrice F
        self.F[0][2] = -self.v * self.deltaT * math.sin(self.theta)
        self.F[0][3] = self.deltaT * math.cos(self.theta)
        self.F[1][2] = self.v * self.deltaT * math.cos(self.theta)
        self.F[1][3] = self.deltaT * math.sin(self.theta)
        self.F[2][4] = self.deltaT
        self.P = self.F @ self.P @ self.F.T + self.Q
        
	# Mise a jour du vecteur d'état
        x_corr = self.x
        y_corr = self.y
        theta_corr = self.theta
        v_corr = self.v
        w_corr = self.w

        self.x = x_corr + v_corr * math.cos(theta_corr)*self.deltaT # Xk+1|k
        self.y = y_corr + v_corr * math.sin(theta_corr)*self.deltaT
        self.theta = theta_corr + w_corr * self.deltaT
        self.v = v_corr
        self.w = w_corr
        
        self.X_etat = np.array([[self.x],[self.y],[self.theta],[self.v],[self.w]])
        
        
        
        self.publish_estimate()
        
        
    def receive_odom(self, odom_msg:Odometry):
    
        self.last_twist = odom_msg.twist.twist  # update twist
        
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z

        self.Z[0][0] = linear_velocity
        self.Z[1][0] = angular_velocity
        
        # ÉTAPE DE CORRECTION
        X_etat_pred = self.X_etat
        p_pred = self.P
        k_pred = self.K
        
        self.X_etat = X_etat_pred + k_pred @ (self.Z-self.C @ X_etat_pred) # Xk|k
        self.P = (np.eye(5) - (k_pred @ self.C)) @ p_pred
        self.K = p_pred @ self.C.T @ np.linalg.inv(self.C @ p_pred @ self.C.T + self.R)

        
    def receive_imu(self, imu_msg:Imu):
        linear_acceleration = imu_msg.linear_acceleration # Attention, contient .x .y et .z
        angular_velocity = imu_msg.angular_velocity.z
        
        # ÉTAPE DE CORRECTION
        # Non utilisé
        
    
    # À adapter
    def publish_estimate(self):
        
        x = self.X_etat[0][0]
        y = self.X_etat[1][0]
        theta = self.X_etat[2][0]
        
        covariance = self.P[0:3,0:3]
        #covariance = np.zeros((3,3)) # Il ne faut que la partie de la covariance concernant x, y et theta
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # 0 ?
        #print(msg.pose.pose.position.y)
      
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)

        msg.pose.covariance[ 0] = covariance[0][0]
        msg.pose.covariance[ 1] = covariance[0][1]
        msg.pose.covariance[ 5] = covariance[0][2]
        msg.pose.covariance[ 6] = covariance[1][0]
        msg.pose.covariance[ 7] = covariance[1][1]
        msg.pose.covariance[11] = covariance[1][2]
        msg.pose.covariance[30] = covariance[2][0]
        msg.pose.covariance[31] = covariance[2][1]
        msg.pose.covariance[35] = covariance[2][2]
        
        print('msg_x :',msg.pose.pose.position.x)
        print('msg_y :',msg.pose.pose.position.y)
        print('msg_theta :',theta)
        self.estimate_publisher.publish(msg)
    
    
    def run(self):
        rospy.spin()
        
        
if __name__ == "__main__":
    node = StateEstimation()
    node.run()

