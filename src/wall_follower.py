#!/usr/bin/env python

import rospy
from math import  pi
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()

    def scan_callback(self, msg):
        # Assume the wall is on the right
        front_distance = min(msg.ranges[0:30] + msg.ranges[330:359])
        right_distance = min(msg.ranges[60:120])
        print(front_distance)
        print(right_distance)

        # Control logic to follow the wall
        if front_distance < 0.3:  # Too close to the wall in front
            self.twist.linear.x = 0.1
            self.twist.angular.z = pi/4  # Turn left
        elif right_distance > 0.2:  # Too far from the wall on the right
            self.twist.linear.x = 0.1
            self.twist.angular.z = -0.3  # Turn right
        else:  # Just right
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.cmd_pub.publish(self.twist)
        
if __name__ == '__main__':
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
