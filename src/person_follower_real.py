#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi


class Object_Follower:
    def __init__(self):
        rospy.init_node('object follower')

        # Publisher to send commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the laser scan topic
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Set the value
        self.front_threshold = 0.2 
        self.forward_speed = 0.2 
        self.turn_speed = pi/8 

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
            front_distance = min(ranges[0:30]+ranges[-30:]) 
            object_distance = min(ranges)
            break
        
        # Adjust movement based on the sensor readings
        self.object_following_logic (front_distance, object_distance)

    def object_following_logic(self, front_distance, object_distance):
        move_cmd = Twist()

        # Turn to the direction of the object
        if front_distance != object_distance:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = self.turn_speed
        
        # Prevent the bot to hit the object
        elif front_distance < self.front_threshold:
            move_cmd.linear.x = 0.0

        else:
            move_cmd.linear.x = self.forward_speed
            move_cmd.angular.z = 0.0


        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    object_follower = Object_Follower()
    object_follower.run()