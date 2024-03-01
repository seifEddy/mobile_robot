#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

class PDController:
    def __init__(self, kp_linear, kd_linear, kp_angular, kd_angular, desired_linear_vel, desired_angular_vel):
        self.kp_linear = kp_linear
        self.kd_linear = kd_linear
        self.kp_angular = kp_angular
        self.kd_angular = kd_angular
        self.desired_linear_vel = desired_linear_vel
        self.desired_angular_vel = desired_angular_vel
        self.prev_linear_error = 0
        self.prev_angular_error = 0
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.init_node('pd_controller', anonymous=True)

    def odom_callback(self, msg):
        current_linear_vel = msg.twist.twist.linear.x
        current_angular_vel = msg.twist.twist.angular.z

        linear_error = self.desired_linear_vel - current_linear_vel
        angular_error = self.desired_angular_vel - current_angular_vel

        linear_derivative = linear_error - self.prev_linear_error
        linear_control = self.kp_linear * linear_error + self.kd_linear * linear_derivative

        angular_derivative = angular_error - self.prev_angular_error
        angular_control = self.kp_angular * angular_error + self.kd_angular * angular_derivative

        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error

        new_cmd = Twist()
        new_cmd.linear.x = linear_control
        new_cmd.angular.z = angular_control
        # print(new_cmd)
        self.cmd_pub.publish(new_cmd)
        rospy.sleep(0.05)

if __name__ == '__main__':
    
    kp_linear = 0.5
    kd_linear = 0.1
    kp_angular = 0.5
    kd_angular = 0.1
    desired_linear_vel = float(input("Desired linear velocity: "))
    desired_angular_vel = float(input("Desired angular velocity: "))

    controller = PDController(kp_linear, kd_linear, kp_angular, kd_angular, desired_linear_vel, desired_angular_vel)
    rospy.spin()
