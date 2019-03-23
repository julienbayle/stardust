#!/usr/bin/env python

import rospy
import math
import time
from std_msgs.msg import Int16, Int32

class Calibrate:

    def __init__(self):
      self.speed = 0
      self.summary = []

    def encoder_cb(self, encoder_value):
      self.speed = encoder_value.data
      self.ready = True
      if self.speed <> 0:
        self.speed_count += 1
        rospy.loginfo("Speed is positive : %f (%d)" % (self.speed, self.speed_count))

    def search(self, pwm_min, pwm_step, pwm_max):
      pwm = pwm_min;

      r = rospy.Rate(1)
      self.ready = False
      while not (rospy.is_shutdown() or self.ready):
        rospy.loginfo("Waiting for encoder data")
        r.sleep()

      r = rospy.Rate(2)
      self.speed_count = 0
      while not (rospy.is_shutdown() or self.speed_count > 10 or pwm > pwm_max):
        pwm += pwm_step
        self.pwmPub.publish(pwm)
        self.speed_count = 0
        rospy.loginfo("PWM: %d - SPEED %d", pwm, self.speed)
        r.sleep()

      self.pwmPub.publish(0)

      return pwm

    def run(self, wheel_speed_topic, wheel_pwm_topic, pwm_min, pwm_max):
        rospy.loginfo("Calibration is starting for %s...", wheel_pwm_topic)
        
        rospy.Subscriber(wheel_speed_topic, Int32, self.encoder_cb)
        self.pwmPub = rospy.Publisher(wheel_pwm_topic, Int16, queue_size=10)

        first_step = 100
        pwm = self.search(pwm_min, first_step, pwm_max)
        rospy.loginfo("First evaluation : %d", pwm)
        self.summary.append("(1) %s -> %d" % (wheel_pwm_topic, pwm))
	
	time.sleep(2)
        
        pwm = self.search(pwm-first_step,10, pwm+first_step)
        rospy.loginfo("Second evaluation : %d", pwm)
        self.summary.append("(2) %s -> %d" % (wheel_pwm_topic, pwm))
      
        rospy.loginfo("Calibration process done for %s...", wheel_pwm_topic)

    def print_summary(self):
        for s in self.summary:
          print(s)

if __name__ == '__main__':
    rospy.init_node('calibration')
    calibrate = Calibrate()
    calibrate.run('/r1/encoder/right/speed', '/r1/pwm/right', 1000, 4000)
    calibrate.run('/r1/encoder/left/speed', '/r1/pwm/left', 1000, 4000)
    calibrate.run('/r1/encoder/back/speed', '/r1/pwm/back', 1000, 4000)
    calibrate.print_summary()
