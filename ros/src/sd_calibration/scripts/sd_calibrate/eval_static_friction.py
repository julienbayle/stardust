#!/usr/bin/env python

import sys
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

      r = rospy.Rate(1)
      self.speed_count = 0
      while not (rospy.is_shutdown() or self.speed_count > 2 or pwm > pwm_max):
        pwm += pwm_step
        self.pwmPub.publish(pwm)
        self.speed_count = 0
        rospy.loginfo("PWM: %d - SPEED %d", pwm, self.speed)
        r.sleep()

      self.pwmPub.publish(0)

      return pwm

    def run(self, wheel_speed_topic, wheel_pwm_topic, pwm_min, pwm_max):
        rospy.loginfo("Calibration is starting for %s...", wheel_pwm_topic)
        
        rospy.Subscriber(wheel_speed_topic, Int16, self.encoder_cb)
        rospy.Subscriber(wheel_speed_topic, Int32, self.encoder_cb)
        self.pwmPub = rospy.Publisher(wheel_pwm_topic, Int16, queue_size=10)

        first_step = (pwm_max-pwm_min) / 10
        pwm = self.search(pwm_min, first_step, pwm_max)
        rospy.loginfo("First evaluation of static friction: %d", pwm)
        self.summary.append("(1) %s -> %d" % (wheel_pwm_topic, pwm))
	
        time.sleep(2)
        
        second_step = 4 * first_step / 10
        pwm = self.search(pwm - 2*first_step, second_step, pwm + 2* first_step)
        rospy.loginfo("Second evaluation of static friction: %d", pwm)
        self.summary.append("(2) %s -> %d" % (wheel_pwm_topic, pwm))
      
        rospy.loginfo("Calibration process done for %s...", wheel_pwm_topic)

    def print_summary(self):
        for s in self.summary:
          print(s)

if __name__ == '__main__':
    rospy.init_node('evalstaticfriction')
    calibrate = Calibrate()
    if len(sys.argv) != 5:
      print("Usage : eval_static_friction.py encoder_speed_topic effort_topic min_effort max_effort")
    else:
      min_effort = int(sys.argv[3])
      max_effort = int(sys.argv[4])
      calibrate.run(sys.argv[1], sys.argv[2], min_effort, max_effort)
      calibrate.print_summary()
