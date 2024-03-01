#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import math
import time

class Controller:
    def __init__(self):
        rospy.init_node('controler_node', anonymous=True)

        rospy.Subscriber('motor1_encoder', Int32, self.motor1_cb)
        rospy.Subscriber('motor2_encoder', Int32, self.motor2_cb)
        rospy.Subscriber('motor3_encoder', Int32, self.motor3_cb)
        rospy.Subscriber('motor4_encoder', Int32, self.motor4_cb)
        
        self.v_pub = rospy.Publisher('v', Float32, queue_size=1)
        self.w_pub = rospy.Publisher('w', Float32, queue_size=1)
        
        self.L = 0.435
        self.counts_per_rev = 140
        self.wheel_radius = 0.055

        self.motor1 = 0
        self.motor2 = 0
        self.motor3 = 0
        self.motor4 = 0

        self.prev_motor1 = 0
        self.prev_motor2 = 0
        self.prev_motor3 = 0
        self.prev_motor4 = 0

        # Velocities
        self.v = 0.0
        self.w = 0.0

        self.v1 = 0.0
        self.v2 = 0.0
        self.v3 = 0.0
        self.v4 = 0.0
        
        self.freq = 25
        self.rate = rospy.Rate(self.freq)
        
        self.update = 0

        # Publishing odometry information.
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

        # Current state of the robot.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time tracking.
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.dt = None
        self.dt1 = None
        self.dt2 = None
        self.dt3 = None
        self.dt4 = None

    def motor1_cb(self, msg):
        if self.dt1:
            self.dt = rospy.Time.now().to_sec() - self.dt1
            self.dt1 = rospy.Time.now().to_sec()
            self.motor1 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
            self.v1 = (self.motor1 - self.prev_motor1) / self.dt
            self.prev_motor1 = self.motor1
            self.update += 1
        else:
            self.dt1 = rospy.Time.now().to_sec()

    def motor2_cb(self, msg):
        if self.dt2:
            self.dt = rospy.Time.now().to_sec() - self.dt2
            self.dt2 = rospy.Time.now().to_sec()
            self.motor2 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
            self.v2 = (self.motor1 - self.prev_motor2) / self.dt
            self.prev_motor2 = self.motor2
            self.update += 1
        else:
            self.dt2 = rospy.Time.now().to_sec()

    def motor3_cb(self, msg):
        if self.dt3:
            self.dt = rospy.Time.now().to_sec() - self.dt3
            self.dt3 = rospy.Time.now().to_sec()
            self.motor3 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
            self.v3 = (self.motor3 - self.prev_motor3) / self.dt
            self.prev_motor2 = self.motor3
            self.update += 1
        else:
            self.dt3 = rospy.Time.now().to_sec()

    def motor4_cb(self, msg):
        if self.dt4:
            self.dt = rospy.Time.now().to_sec() - self.dt4
            self.dt4 = rospy.Time.now().to_sec()
            self.motor4 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
            self.v4 = (self.motor4 - self.prev_motor4) / self.dt
            self.prev_motor4 = self.motor4
            self.update += 1
        else:
            self.dt4 = rospy.Time.now().to_sec()

    def determine_velocity(self):
        self.v = round(0.5 * (self.v1 + self.v4), 3)
        self.w = round((self.v1 - self.v4) / self.L, 3)
        v = Float32()
        w = Float32()
        v.data = self.v
        w.data = self.w
        self.v_pub.publish(v)
        self.w_pub.publish(w)
        # print('V = ' + str(self.v) + ' -****- w = ' + str(self.w))

    def run(self):
        while not rospy.is_shutdown():
            
            if self.update > 3:
                self.update = 0
                self.determine_velocity()
                self.publish_odom()
            
    def publish_odom(self):
        # Update current time and compute time difference.
        if self.dt is None:
            return

        # print(self.dt)
        self.current_time = rospy.Time.now()
        # dt = (self.current_time - self.last_time).to_sec()

        # Extract linear and angular velocities.
        vx = self.v
        vy = 0.0
        vth = self.w

        # Compute the distance and angle moved.self.dt1 is None
        delta_x = (vx * math.cos(self.theta)) * self.dt
        delta_y = (vx * math.sin(self.theta)) * self.dt
        delta_th = vth * self.dt

        # Update the pose.
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Create quaternion from theta.
        odom_quat = quaternion_from_euler(0, 0, self.theta)

        # Create and publish the odometry message.
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # Set the position.
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        # Set the velocity.
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth

        # Publish the message.
        self.odom_pub.publish(odom)

        # Update the last time for the next iteration.
        self.last_time = self.current_time


if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
    
