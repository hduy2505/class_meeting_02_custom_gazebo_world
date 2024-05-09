import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turn:
     def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        
     def turn(self, time):
        self.twist.linear.x = 0.0
        self.twist.angular.z = pi/2 # adjust this value to your needs
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(time)
        
     def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        
     def run(self):
        rospy.init_node('turn')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
                self.turn(1) # turn for 5 seconds
                self.stop() # stop
                rospy.sleep(1) # pause
                rate.sleep()
                
turn = Turn()
turn.run()