#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool


class MrRobot:
    def __init__(self):
        self.velocity_msg = Twist()
        self.feedback_value = 0
        self.line_exists = True
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.kp = 0.004  # rospy.get_param("line_follower_controller/pid/p")
        rospy.Subscriber('/feedback_bool', Bool, self.bool_callback)
        rospy.Subscriber('/feedback', Float32, self.feedback_callback)

    def bool_callback(self, bool_msg):
        if bool_msg.data:
            self.line_exists = True
        else:
            rospy.logwarn("No line is detected")
            self.line_exists = False
            self.move(0,0)

    def feedback_callback(self, feedback_msg):
        # Process the received feedback value
        self.feedback_value = feedback_msg.data

        rospy.loginfo(f"Received feedback value: {self.feedback_value}")
        self.fix_error()

    def move(self, linear, angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)
        rospy.loginfo(f"Publishing linear: {linear}, angular: {angular}")

    def fix_error(self):
        rospy.loginfo(f"Line exists: {self.line_exists}")
        if abs(self.feedback_value) < 5.0:
            # Moving in a straight line
            self.move(0.4, 0)
            rospy.loginfo("Moving straight")
                
        elif self.feedback_value < 0:
            # Fixing the yaw by turning left
            self.move(0.4, -(self.kp * self.feedback_value))
            rospy.loginfo("Fixing yaw by turning left")
                
        elif self.feedback_value > 0:
            # Fixing the yaw by turning right
            self.move(0.4, -(self.kp * self.feedback_value))
            rospy.loginfo("Fixing yaw by turning right")


def main():
    rospy.init_node('mr_robot_node', anonymous=True)
    robot = MrRobot()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
