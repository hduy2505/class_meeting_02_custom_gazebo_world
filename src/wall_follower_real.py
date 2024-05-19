#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower_node')

        # Publisher to send commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the laser scan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Set wall following parameters
        self.wall_threshold = 0.15  # Desired distance to the wall (meters)
        self.front_threshold = 0.3 # Distance threshold for front obstacles (meters)
        self.forward_speed = 0.2  # Speed to move forward
        self.turn_speed = pi/8  # Turning speed

    def scan_callback(self, msg):
        # Extract ranges from the laser scan message
        ranges = list(msg.ranges)
        # Processing data cause in reality infinity value will be show as 0
        i = 0
        while i < 360:
            if ranges[i] == 0:
                ranges[i] = 15
            i += 1
        # To make that only gain the value after change
        while i >= 360:
            right_distance = min(ranges[240:300]) # Scan data on the right side (240° to 300°)
            front_distance = min(ranges[0:30]+ranges[-30:]) # Scan data directly ahead (0° ±30°)
            break


        # Adjust movement based on the sensor readings
        self.wall_following_logic(front_distance, right_distance)

    def wall_following_logic(self, front_distance, right_distance):
        move_cmd = Twist()

        # Obstacle directly ahead - turn left
        if front_distance < self.front_threshold:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = self.turn_speed
            rospy.loginfo("Turning left to avoid obstacle ahead")

        # Too far from the right wall - turn right
        elif right_distance > self.wall_threshold:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = -self.turn_speed / 2  # Mild right turn
            rospy.loginfo("Turning right to follow the wall")

        # Too close to the right wall - turn left
        elif right_distance < self.wall_threshold:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = self.turn_speed / 2  # Mild left turn
            rospy.loginfo("Turning left to maintain distance from the wall")

        # Move forward and stay parallel to the wall
        else:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = 0.0
            rospy.loginfo("Moving forward and following the wall")

        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    wall_follower = WallFollower()
    wall_follower.run()