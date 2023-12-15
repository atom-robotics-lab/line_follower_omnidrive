#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MrRobot:
    def __init__(self):
        self.velocity_msg = Twist()
        self.feedback_value = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.kp = 0.004  # rospy.get_param("line_follower_controller/pid/p")
        rospy.Subscriber('/feedback', Float32, self.feedback_callback)
        self.corrected_yaw = False

    def feedback_callback(self, feedback_msg):
        # Process the received feedback value
        self.feedback_value = feedback_msg.data
        rospy.loginfo(f"Received feedback value: {self.feedback_value}")
        self.fix_error()

    def move(self, linear, angular):
        self.velocity_msg.linear.x = linear
        self.velocity_msg.angular.z = angular
        self.pub.publish(self.velocity_msg)

    def fix_error(self):

        
        if abs(self.feedback_value) < 5:
            self.corrected_yaw = True

        if self.feedback_value < 0:
            # Fixing the yaw by turning left
            self.move(0.4, -(self.kp * self.feedback_value))
            rospy.loginfo("Fixing yaw by turning left")
            self.corrected_yaw = False

        elif self.feedback_value > 0:
            # Fixing the yaw by turning right
            self.move(0.4, -(self.kp * self.feedback_value))
            rospy.loginfo("Fixing yaw by turning right")
            self.corrected_yaw = False

        if(self.corrected_yaw is True):
            # Moving in a straight line
            self.move(0.4, 0)
            rospy.loginfo("Moving straight")

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('mr_robot_node', anonymous=True)

        robot = MrRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
