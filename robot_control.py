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

        self.current_pose = None  # Initialiser la variable qui stocke la pose actuelle
        self.current_scan = None
        self.goal = {'x': -3.0, 'y': 0.0}  # Emplacement cible

    def pose_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_euler(msg.pose.pose.orientation)
        }

    def quaternion_to_euler(self, quat):
        # Conversion des quaternions en angles d'Euler, en supposant que les quaternions sont normalisés
        euler = tf.transformations.euler_from_quaternion([
            quat.x,
            quat.y,
            quat.z,
            quat.w
        ])
        return euler[2]  # Retour de l'angle de lacet

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

        # Calculer la force totale
        #force_x = attract['x'] + repulse['x']
        #force_y = attract['y'] + repulse['y']
        force_x = attract['x']
        force_y = attract['y']
        # Angle cible (radians)
        target_angle = math.atan2(force_y, force_x)
    
        # Orientation actuelle
        current_angle = pose['theta']
        print(current_angle)
        # différence angulaire
        angle_diff = self.normalize_angle(target_angle - current_angle)
        
        # Convertit la force en commande de vitesse
        if abs(angle_diff) > 0.05:
              command.angular.z = 0.15 * angle_diff  # Ajustement du facteur de vitesse de rotation
        else:
              command.angular.z = 0
        
        dist = math.sqrt(force_x**2 + force_y**2)
        if dist > 0.05:
              command.linear.x = 0.05    # Ajustement du facteur de vitesse d'avancement
        else:
              command.linear.x = 0

        return command

    def normalize_angle(self, angle):
        # Angle normalisé par rapport à l'intervalle [-π, π].
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_attractive_force(self, pose):
        # Attractivité en direction de la cible
        force_x = self.goal['x'] - pose['x']
        force_y = self.goal['y'] - pose['y']
        
        norm = math.sqrt(force_x**2 + force_y**2)
        if norm > 0:
            return {'x': force_x, 'y': force_y}
            #return {'x': force_x / norm, 'y': force_y / norm}
        else:
            return {'x': 0, 'y': 0}

    def calculate_repulsive_force(self, scan, pose):
        # Les forces de répulsion sont générées par les obstacles proches
        force_x = 0
        force_y = 0
        for i, distance in enumerate(scan):
            if distance < 1.0:  # Distance de répulsion
                angle = i * (2 * math.pi / len(scan)) - math.pi  # Calculer la direction de l'obstacle
                force_x -= (1.0 - distance) * math.cos(angle)
                force_y -= (1.0 - distance) * math.sin(angle)
        return {'x': force_x, 'y': force_y}


if __name__ == '__main__':
    controller = RobotController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
