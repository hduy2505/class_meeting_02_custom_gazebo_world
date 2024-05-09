#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd_vel = Twist()
        self.wall_distance = 0.5  # Adjust this distance according to your needs

    def scan_callback(self, scan):
        # Find the minimum distance from the LaserScan data
        min_distance = min(scan.ranges)

        if min_distance < self.wall_distance:
            # If the distance is less than the desired distance, stop the robot
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = 0
        else:
            # Otherwise, move the robot forward and adjust its direction to follow the wall
            self.cmd_vel.linear.x = 0.2  # Adjust linear velocity as needed
            self.cmd_vel.angular.z = 0.5  # Adjust angular velocity as needed

        self.pub.publish(self.cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
        wall_follower.run()
    except rospy.ROSInterruptException:
        pass
