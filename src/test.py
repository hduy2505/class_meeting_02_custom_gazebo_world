import rospy
from geometry_msgs.msg import Twist

class SquareMovement:
    def __init__(self):
        rospy.init_node('move_in_square', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angular_speed = 0.5  # Adjust as needed
        self.straight_speed = 0.2  # Adjust as needed
        self.straight_duration = 5.0  # Adjust as needed
        self.turn_duration = 2.0  # Adjust as needed
        self.correction_duration = 1.0  # Adjust as needed

    def move_forward(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.straight_speed
        self.vel_pub.publish(vel_msg)
        rospy.sleep(self.straight_duration)

    def stop(self):
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
        rospy.sleep(1.0)

    def turn(self):
        vel_msg = Twist()
        vel_msg.angular.z = self.angular_speed
        self.vel_pub.publish(vel_msg)
        rospy.sleep(self.turn_duration)

    def correct_angle(self):
        # Turn in the opposite direction for a fixed duration to correct angle deviation
        vel_msg = Twist()
        vel_msg.angular.z = -self.angular_speed
        self.vel_pub.publish(vel_msg)
        rospy.sleep(self.correction_duration)

    def move_in_square(self):
        while not rospy.is_shutdown():
            self.move_forward()
            self.stop()
            self.turn()
            self.stop()
            self.correct_angle()
            self.stop()

square_movement = SquareMovement()
square_movement.move_in_square()