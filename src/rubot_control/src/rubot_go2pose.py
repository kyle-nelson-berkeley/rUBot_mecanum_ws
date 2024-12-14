##!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class RubotGoToPose:
    def __init__(self):
        rospy.init_node('rubot_go2pose', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = None
        self.is_stuck = False
        self.target_x = 2.0  # Example target x-coordinate
        self.target_y = 2.0  # Example target y-coordinate

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.check_if_stuck()

    def check_if_stuck(self):
        # Example simple stuck detection logic
        if self.current_pose and self.current_pose.position.x < 0.1:  # Adjust as needed
            rospy.loginfo("Robot seems to be stuck! Attempting to move back.")
            self.is_stuck = True
            self.attempt_recovery()

    def attempt_recovery(self):
        twist = Twist()
        twist.linear.x = -0.2  # Move backward slightly
        self.vel_pub.publish(twist)
        rospy.sleep(2)  # Adjust sleep duration as needed
        twist.linear.x = 0.0
        self.vel_pub.publish(twist)
        rospy.loginfo("Recovery attempt complete. Resuming motion.")
        self.is_stuck = False

    def calculate_angle_to_target(self):
        # Calculate angle to the target from the current position
        if self.current_pose is None:
            return 0.0

        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        angle_to_goal = math.atan2(dy, dx)

        # Extract current orientation as yaw
        orientation_q = self.current_pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        return angle_to_goal - yaw

    def move_to_goal(self):
        rate = rospy.Rate(10)  # 10 Hz
        twist = Twist()
        while not rospy.is_shutdown():
            if self.is_stuck:
                rate.sleep()
                continue  # Skip movement commands if in recovery

            # Calculate the angle to the target
            angle_to_target = self.calculate_angle_to_target()

            if abs(angle_to_target) > 0.1:  # Adjust threshold for turning
                twist.linear.x = 0.0
                twist.angular.z = 0.3 if angle_to_target > 0 else -0.3  # Rotate towards the target
            else:
                twist.linear.x = 0.5  # Move forward if aligned with target
                twist.angular.z = 0.0

            self.vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rubot = RubotGoToPose()
        rubot.move_to_goal()
    except rospy.ROSInterruptException:
        pass



# OLD FUNCTION
# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from math import pow, atan2, sqrt, radians, degrees
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# class Rubot:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('rubot_control', anonymous=True)

#         # Retrieve goal parameters
#         self.x_goal = rospy.get_param("~x", 0.0)
#         self.y_goal = rospy.get_param("~y", 0.0)
#         f_goal_deg = rospy.get_param("~f", 0.0)
#         self.f_goal = radians(f_goal_deg)
#         #q = quaternion_from_euler(0, 0, self.f_goal)
#         #self.q_goal.x = q[0]
#         #self.q_goal.y = q[1]
#         #self.q_goal.z = q[2]
#         #self.q_goal.w = q[3]

#         # Initialize current pose variables
#         self.x_pose = 0.0
#         self.y_pose = 0.0
#         self.yaw = 0.0

#         # Publisher for velocity commands
#         self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         # Subscriber for odometry updates
#         self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)

#         self.odom = Odometry()
#         self.rate = rospy.Rate(10)

#         # Set up shutdown handler
#         rospy.on_shutdown(self.shutdown_handler)

#     def update_odom(self, data):
#         """Callback function for updating the robot's current odometry."""
#         self.odom = data
#         self.x_pose = round(self.odom.pose.pose.position.x, 2)
#         self.y_pose = round(self.odom.pose.pose.position.y, 2)
#         orientation_q = self.odom.pose.pose.orientation
#         orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#         self.yaw = euler_from_quaternion(orientation_list)[2]

#     def euclidean_distance(self):
#         """Calculate the Euclidean distance between the current pose and the goal."""
#         return sqrt(pow((self.x_goal - self.x_pose), 2) + pow((self.y_goal - self.y_pose), 2))

#     def linear_vel(self, constant=0.5):
#         """Calculate the linear velocity towards the goal."""
#         return constant * self.euclidean_distance()

#     def steering_angle(self):
#         """Calculate the steering angle towards the goal."""
#         return atan2(self.y_goal - self.y_pose, self.x_goal - self.x_pose)

#     def angular_vel(self, constant=5):
#         """Calculate the angular velocity towards the goal."""
#         return constant * (self.steering_angle() - self.yaw)

#     def move_to_pose(self):
#         """Move the robot towards the goal pose."""
#         distance_tolerance = 0.1
#         angle_tolerance = 0.1

#         vel_msg = Twist()

#         # Move towards the goal position
#         while not rospy.is_shutdown() and self.euclidean_distance() >= distance_tolerance:
#             # Linear velocity in the x-axis.
#             vel_msg.linear.x = self.linear_vel()
#             vel_msg.linear.y = 0
#             vel_msg.linear.z = 0

#             # Angular velocity in the z-axis.
#             vel_msg.angular.x = 0
#             vel_msg.angular.y = 0
#             vel_msg.angular.z = self.angular_vel()

#             # Publish velocity message
#             self.velocity_publisher.publish(vel_msg)
#             rospy.loginfo("Distance to target: %.2f", self.euclidean_distance())

#             # Maintain the loop rate
#             self.rate.sleep()

#         # Align with the final orientation
#         while not rospy.is_shutdown() and abs(self.f_goal - self.yaw) >= angle_tolerance:
#             vel_msg.linear.x = 0
#             vel_msg.angular.z = (self.f_goal - self.yaw) * 0.5

#             # Publish velocity message
#             self.velocity_publisher.publish(vel_msg)
#             rospy.loginfo("Orientation error: %.2f", abs(degrees(self.f_goal - self.yaw)))

#             # Maintain the loop rate
#             self.rate.sleep()

#         # Stop the robot
#         self.stop_robot()

#     def stop_robot(self):
#         """Send a zero velocity command to stop the robot."""
#         vel_msg = Twist()
#         vel_msg.linear.x = 0
#         vel_msg.angular.z = 0
#         self.velocity_publisher.publish(vel_msg)
#         rospy.loginfo("Robot stopped.")

#     def shutdown_handler(self):
#         """Handle the shutdown signal by stopping the robot."""
#         self.stop_robot()
#         rospy.loginfo("Shutdown initiated. Robot stopped.")

# if __name__ == '__main__':
#     try:
#         rubot = Rubot()
#         rubot.move_to_pose()
#     except rospy.ROSInterruptException:
#         rubot.shutdown_handler()