#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Float32

PI = 3.1415926535897

class MrRobot:
    def __init__(self, tx, ty):
        self.current_position = (0, 0)
        self.tx = tx
        self.ty = ty
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.ad = 0
        self.feedback_value = 0
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/feedback', Float32, self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        # Process the received feedback value
        self.feedback_value = feedback_msg.data
        rospy.loginfo(f"Received feedback value: {self.feedback_value}")
        self.move_to_point()


    def odom_callback(self, odom_msg):
        # Update the current position based on odometry information
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

        # Convert quaternion to Euler angles
        _, _, self.yaw = euler_from_quaternion([
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ])

        # Convert yaw to degrees
        self.yaw = math.degrees(self.yaw)

        rospy.loginfo(f"Current position: ({self.x}, {self.y}), Yaw: {self.yaw}")

    def move_to_point(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        vel_msg = Twist()
        speed = 30
        angular_speed = speed * 2 * PI / 360
        rospy.loginfo("ohwgofwh-fgwu")

        if  (abs(self.feedback_value)) > 5:
            # Adjust the tolerance as needed

            # Stop linear motion
            vel_msg.linear.x = 0.0
            rospy.loginfo("robot is rotation")

            # Set the angular velocity to align with the desired angle
            if self.feedback_value < 0:
                vel_msg.angular.z = abs(angular_speed)
            else:
                vel_msg.angular.z = -abs(angular_speed)
            pub.publish(vel_msg)
            rate.sleep()

        # Stop the robot when the desired orientation is reached
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
        rospy.loginfo("Reached the desired orientation.")

        # Moving in linear velocity
        linear_speed = 0.2  # Adjust as needed
        vel_msg.linear.x = linear_speed

        
        pub.publish(vel_msg)
        rate.sleep()

        distance_to_goal = math.sqrt((self.tx - self.x) ** 2 + (self.ty - self.y) ** 2)
        rospy.loginfo(f"Distance to goal: {distance_to_goal}")

        if distance_to_goal < 0.2:
            vel_msg.linear.x = 0.0
            pub.publish(vel_msg)
            rospy.loginfo("Robot reached the goal.")

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node('mr_robot_node', anonymous=True)

        # Create an instance of the Robot class with default target coordinates (0, 0)
        my_robot = MrRobot(tx=-0.233497, ty=3)
        my_robot.move_to_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
