#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)

        # 目标位置
        self.goal = Point()
        self.goal.x = rospy.get_param('~goal_x', 5.0)
        self.goal.y = rospy.get_param('~goal_y', 0.0)

        # 机器人当前位置
        self.position = Point()
        self.yaw = 0.0

        # 激光雷达数据
        self.laser_min = 0.0
        self.laser_ranges = []

        # 记录当前的转向方向
        self.turning_direction = None

        # 订阅必要的话题
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # 发布控制指令
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
        left_distance_old = -1
        yaw_old = 0
        x_old = 0
        y_old = 0
        turning = False
        left_cross_detected = False

        while not rospy.is_shutdown():
            if self.laser_ranges:
                front_distance = min(min(self.laser_ranges[0:25]),  min(self.laser_ranges[-25:]))  # 前方距离
                left_distance = min(self.laser_ranges[25:90])  # 左侧距离
                right_distance = min(self.laser_ranges[-90:-25])  # 右侧距离

                left_front = min(self.laser_ranges[30:60])
                left_back = min(self.laser_ranges[120:150])
                left_cross_dector = min(self.laser_ranges[45:90])
                left_side_distance = min(self.laser_ranges[80:110])
                distance_passed = math.sqrt((x_old-self.position.x)**2 + (y_old - self.position.y)**2)

                left_vertical_angel_diff = self.laser_ranges.index(min(self.laser_ranges[60:120])) - 90

                max_left_distance = max(self.laser_ranges[30:90])  # 左侧距离
                max_right_distance = max(self.laser_ranges[-90:-30])  # 右侧距离

                print(left_cross_dector)

                if front_distance < 0.25 and not turning:
                    # 遇到障碍物，选择转弯方向
                    if self.turning_direction is None:
                        if right_distance < 0.3 and max_left_distance >  max_right_distance:
                            self.turning_direction = 'left'
                            twist.angular.z = 0.3  # 左转
                            print("jin zuo")
                        else:# left_distance < 0.25:
                            self.turning_direction = 'right'
                            twist.angular.z = -0.3  # 右转
                            print("jin you")
                        # elif left_distance > right_distance:
                        #     self.turning_direction = 'left'
                        #     twist.angular.z = 0.5  # 左转       
                        # else:
                        #     self.turning_direction = 'right'
                        #     twist.angular.z = -0.5  # 右转
                    else:
                        # 保持当前转向方向
                        twist.angular.z = 0.5 if self.turning_direction == 'left' else -0.5

                    twist.linear.x = 0.0
                elif left_cross_detected and distance_passed > 0.05:
                    turning = True
                    distance_passed = 0
                    left_cross_detected = False
                    twist.linear.x = 0

                elif turning and abs(yaw_old - self.yaw) > 0.2:
                    twist.angular.z = 0.5  # 左转
                    twist.linear.x = 0.0

                elif left_cross_dector > 0.1 + left_distance_old and left_distance_old > 0 and not left_cross_detected:
                    print(1)
                    self.turning_direction = None
                    left_cross_detected = True
                    yaw_old = self.yaw + math.pi/2
                    x_old = self.position.x
                    y_old = self.position.y
                    if yaw_old >= math.pi:
                        yaw_old -= 2*math.pi
                    if yaw_old <= -math.pi:
                        yaw_old += 2*math.pi

                else:
                    # 清除转向方向
                    self.turning_direction = None
                    turning = False

                    # 朝目标位置前进
                    distance = self.get_distance_to_goal()
                    angle = self.get_angle_to_goal()
                    angle_diff = angle - self.yaw

                    if distance > 0.1:
                        twist.linear.x = 0.2
                        if left_vertical_angel_diff > 0 and left_vertical_angel_diff < 15 and left_side_distance < 0.2 or abs(left_side_distance - 0.35) < 0.1:
                            twist.angular.z = 0.3
                        elif left_vertical_angel_diff < 0 and left_vertical_angel_diff > -15 and left_side_distance < 0.2:
                            twist.angular.z = -0.2
                        else:
                            twist.angular.z = 0
                        # if left_distance < 0.15:
                        #     twist.angular.z = -0.5  # 右转
                        # elif right_distance < 0.15:
                        #     twist.angular.z = 0.5  # 左转
                        # else:
                        #     twist.angular.z = 0.5 * angle_diff
                    else:
                        # 到达目标
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist)
                        rospy.loginfo("Reached the goal!")
                        break
                left_distance_old = left_cross_dector
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        maze_solver = MazeSolver()
        maze_solver.move_to_goal()
    except rospy.ROSInterruptException:
        pass
