#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32
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

    def motor1_cb(self, msg):
        self.motor1 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
        self.update += 1

    def motor2_cb(self, msg):
        self.motor2 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
        self.update += 1

    def motor3_cb(self, msg):
        self.motor3 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
        self.update += 1

    def motor4_cb(self, msg):
        self.motor4 = msg.data * 2 * math.pi * self.wheel_radius / self.counts_per_rev
        self.update += 1

    def determine_velocity(self):
        self.v = 0.5 * (self.v1 + self.v4)
        self.w = (self.v1 - self.v4) / self.L
        v = Float32()
        w = Float32()
        v = self.v
        w = self.w
        self.v_pub.publish(v)
        self.w_pub.publish(w)
        # print('V = ' + str(self.v) + ' -****- w = ' + str(self.w))

    def run(self):
        t0 = time.time()
        while not rospy.is_shutdown():
            
            if self.update > 3:
                self.update = 0

                dt = time.time() - t0
                # print(dt)
                self.v1 = (self.motor1 - self.prev_motor1) / dt
                self.v2 = (self.motor2 - self.prev_motor2) / dt
                self.v3 = (self.motor3 - self.prev_motor3) / dt
                self.v4 = (self.motor4 - self.prev_motor4) / dt
                
                self.prev_motor1 = self.motor1
                self.prev_motor2 = self.motor2
                self.prev_motor3 = self.motor3
                self.prev_motor4 = self.motor4
                t0 = time.time()
                # print(self.v1)
            self.determine_velocity()
            # self.rate.sleep()
            
            


if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
    
