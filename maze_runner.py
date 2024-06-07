#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)

        self.goal = Point()
        self.goal.x = rospy.get_param('~goal_x', 5.0)
        self.goal.y = rospy.get_param('~goal_y', 0.0)
        self.theta = 0

        self.position = Point()
        self.yaw = 0.0

        self.laser_min = 0.0
        self.laser_ranges = []

        self.turning_direction = None

        #rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/estimation', PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def laser_callback(self, msg):
        self.laser_min = msg.range_min
        self.laser_ranges = msg.ranges

    def get_distance_to_goal(self):
        dx = self.goal.x - self.position.x
        dy = self.goal.y - self.position.y
        return math.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self):
        dx = self.goal.x - self.position.x
        dy = self.goal.y - self.position.y
        return math.atan2(dy, dx)

    def move_to_goal(self):
        twist = Twist()
        right_distance_old = -1
        yaw_old = 0
        x_old = 0
        y_old = 0
        avoid_enabled = True
        turning = False
        right_cross_detected = False

        while not rospy.is_shutdown():
            if self.laser_ranges:
                front_distance = min(min(self.laser_ranges[0:30]),  min(self.laser_ranges[-30:]))
                left_distance = min(self.laser_ranges[30:80]) 
                right_distance = min(self.laser_ranges[-80:-30]) 
                out_maze_detector = min(min(self.laser_ranges[0:20]),  min(self.laser_ranges[-20:]))

                left_front = min(self.laser_ranges[30:60])
                left_back = min(self.laser_ranges[120:150])
                left_cross_dector = min(self.laser_ranges[45:90])
                right_cross_dector = min(self.laser_ranges[-90:-45])
                left_side_distance = min(self.laser_ranges[80:110])
                right_side_distance = min(self.laser_ranges[-110:-80])
                distance_passed = math.sqrt((x_old-self.position.x)**2 + (y_old - self.position.y)**2)

                right_vertical_angel_diff = self.laser_ranges.index(min(self.laser_ranges[-120:-60])) - 270

                max_left_distance = max(self.laser_ranges[30:90])
                max_right_distance = max(self.laser_ranges[-90:-30])

                if front_distance > 1:
                    print("out maze")
                    distance = self.get_distance_to_goal()
                    angle = self.get_angle_to_goal()
                    angle_diff = angle - self.yaw

                    if distance > 0.1:
                        twist.linear.x = 0.2
                        twist.angular.z = 0.5 * angle_diff
                    elif abs(self.theta - self.yaw) > 0.1:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        rospy.loginfo("Reached the goal!")
                        break
                else:
                    if front_distance < 0.25 and avoid_enabled:
                        if self.turning_direction is None:
                            if left_distance < 0.3 and max_left_distance <  max_right_distance:
                                self.turning_direction = 'right'
                                twist.angular.z = -0.5
                            else:
                                self.turning_direction = 'left'
                                twist.angular.z = 0.5
                        else:
                            twist.angular.z = 0.5 if self.turning_direction == 'left' else -0.5

                        twist.linear.x = 0.0
                    elif right_cross_detected and distance_passed > 0.05:
                        turning = True
                        distance_passed = 0
                        right_cross_detected = False
                        self.turning_direction = None
                        twist.linear.x = 0

                    elif turning and abs(yaw_old - self.yaw) > 0.2:
                        avoid_enabled = False
                        self.turning_direction = None
                        twist.angular.z = -0.5
                        twist.linear.x = 0.0

                    elif right_cross_dector > 0.1 + right_distance_old and right_distance_old > 0 and not right_cross_detected:
                        avoid_enabled = False
                        self.turning_direction = None
                        right_cross_detected = True
                        yaw_old = self.yaw - math.pi/2
                        x_old = self.position.x
                        y_old = self.position.y
                        if yaw_old >= math.pi:
                            yaw_old -= 2*math.pi
                        if yaw_old <= -math.pi:
                            yaw_old += 2*math.pi

                    else:
                        self.turning_direction = None
                        avoid_enabled = True
                        turning = False

                        distance = self.get_distance_to_goal()
                        angle = self.get_angle_to_goal()
                        angle_diff = angle - self.yaw

                        if distance > 0.1:
                            twist.linear.x = 0.2

                            if right_vertical_angel_diff > 0 and right_side_distance < 0.2 or right_side_distance < 0.13:
                                twist.angular.z = 0.4
                            elif right_vertical_angel_diff < 0 and right_side_distance < 0.2 and right_side_distance > 0.13:
                                twist.angular.z = -0.3
                            else:
                                twist.angular.z = 0

                            # mid = (left_distance + right_distance)/2
                            # if left_distance < mid and left_distance < 0.25:
                            #     twist.angular.z = -0.3
                            # elif right_distance < mid and right_distance < 0.25:
                            #     twist.angular.z = 0.3
                            # else:
                            #     twist.angular.z = 0
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.cmd_vel_pub.publish(twist)
                            rospy.loginfo("Reached the goal!")
                            break
                right_distance_old = right_cross_dector
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        maze_solver = MazeSolver()
        maze_solver.move_to_goal()
    except rospy.ROSInterruptException:
        pass
