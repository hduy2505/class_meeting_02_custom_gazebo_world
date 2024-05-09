#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.angle_ahead =  0
        self.front_distance = 0
        self.range_ahead = 1 # anything to start
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def scan_callback(self, msg):
        # Calculate the minimum range and its angle
        self.front_distance = min(msg.ranges[0:30] + msg.ranges[330:359])         
        min_range = min(msg.ranges)
        min_angle = msg.ranges.index(min_range) * msg.angle_increment

        self.range_ahead = min_range
        self.angle_ahead = min_angle

    def run(self):
        rospy.init_node('follower')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.range_ahead < 0.8: # person is too close
                self.twist.linear.x = 0
            else:
                self.twist.angular.z = 1
                if self.range_ahead == self.front_distance: #detect if the object is at the front 
                    self.twist.linear.x = 0.5
                    self.twist.angular.z = 0
                
                                

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

follower = Follower()
follower.run()
