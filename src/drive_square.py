#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pi

class SquareBot:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def move_forward(self, time):
        self.twist.linear.x = 0.5 
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(time)

    def turn(self, time):
        self.twist.linear.x = 0.0
        self.twist.angular.z = pi/2 # tunr the robot 90 degree 
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(time)

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rospy.init_node('square_bot')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for _ in range(4): # repeat four times
                self.move_forward(5) #move in 5 seconds
                self.stop() 
                rospy.sleep(1) 
                self.turn(1)  #turn 90 degrees 
                self.stop() 
                rospy.sleep(1) 
            rate.sleep()

square_bot = SquareBot()
square_bot.run()
