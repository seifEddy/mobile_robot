#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

'''
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
'''
# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the trigger and echo
TRIG = 16  # Change this value based on your wiring
ECHO = 26  # Change this value based on your wiring

# Set up the GPIO channels - one input and one output
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    # Ensure the trigger pin is low for a clean start
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    # Trigger the ultrasonic pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs
    GPIO.output(TRIG, False)
    pulse_start = None
    pulse_end = None
    # Record the last low timestamp for ECHO (start) and the first high timestamp for ECHO (arrival)
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    if pulse_start and pulse_end:
      # Calculate the duration of the pulse and compute the distance
      pulse_duration = pulse_end - pulse_start
      distance = pulse_duration * 171.50  # Speed of sound in cm/s divided by 2 (there and back)
      distance = round(distance, 2)  # Round to two decimal places
		
      return distance
    else:
      distance = 2.0

try:
    rospy.init_node('range_node', anonymous=True)
    range_pub = rospy.Publisher('range', Range, queue_size=10)
    range_msg = Range()
    range_msg.header.frame_id = 'sonar_link'
    range_msg.radiation_type = range_msg.ULTRASOUND
    range_msg.field_of_view = 0.122
    range_msg.min_range = 0.05
    range_msg.max_range = 2.0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        range_msg.range = measure_distance()
        range_pub.publish(range_msg)
        # print(f"Distance: {dist} cm")
        rate.sleep()
except KeyboardInterrupt:
    # Clean up the GPIO pins before exiting
    GPIO.cleanup()

