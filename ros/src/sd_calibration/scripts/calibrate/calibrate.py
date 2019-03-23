#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int16, Int32

class Calibrate:

    def __init__(self):
      self.speed = 0

    def encoder_cb(self, encoder_value):
      self.speed = encoder_value.data
      self.ready = True
      rospy.loginfo("Calibration / Data : %f" % self.speed)

    def search(self, pwm_min, pwm_step, pwm_max):
      pwm = pwm_min;

      r = rospy.Rate(1)
      self.ready = False
      while not (rospy.is_shutdown() or self.ready):
        rospy.loginfo("Waiting for encoder data")
        r.sleep()

      r = rospy.Rate(10)
      while not (rospy.is_shutdown() or self.speed > 0 or pwm > pwm_max):
        pwm += pwm_step
        self.pwmPub.publish(pwm)
        rospy.loginfo("PWM: %d - SPEED %d", pwm, self.speed)
        r.sleep()

      self.pwmPub.publish(0)

      return pwm

    def run(self, wheel_speed_topic, wheel_pwm_topic, pwm_min, pwm_max):
        rospy.loginfo("Calibration is starting for %s...", wheel_pwm_topic)
        
        rospy.Subscriber(wheel_speed_topic, Int32, self.encoder_cb)
        self.pwmPub = rospy.Publisher(wheel_pwm_topic, Int16, queue_size=10)

        first_step = (pwm_max-pwm_min)/20
        pwm = self.search(pwm_min, first_step, pwm_max)
        rospy.loginfo("First evaluation : %d", pwm)
        
        pwm = self.search(pwm-first_step,(pwm_max-pwm_min)/100, pwm+first_step)
        rospy.loginfo("Second evaluation : %d", pwm)
      
        rospy.loginfo("Calibration process done for %s...", wheel_pwm_topic)

if __name__ == '__main__':
    rospy.init_node('calibration')
    calibrate = Calibrate()
    calibrate.run('/r1/encoder/right/speed', '/r1/pwm/right', 0, 2000)
    calibrate.run('/r1/encoder/left/speed', '/r1/pwm/left', 0, 2000)
    calibrate.run('/r1/encoder/back/speed', '/r1/pwm/back', 0, 2000)