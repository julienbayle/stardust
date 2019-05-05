#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Robot:

    def __init__(self, auto_topic, cmd_vel_topic):
      self.auto_pub = rospy.Publisher(auto_topic, Bool, queue_size=10)
      self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
      
    def move(self, twist_array):
      r = rospy.Rate(100)
      i = 0
      hz = 0
      while (not rospy.is_shutdown()) and i < len(twist_array) :
        twist = twist_array[i]
        if hz == 0:
          rospy.loginfo("Move %f-%f-%f" % (twist.linear.x, twist.linear.y, twist.angular.z))
        self.auto_pub.publish(Bool(True))
        self.cmd_pub.publish(twist)
        r.sleep()
        hz = hz + 1
        if hz == 100:
          hz = 0
          i = i+1
      
      self.auto_pub.publish(Bool(False))
        
if __name__ == '__main__':
  
    rospy.init_node('robot')
    if len(sys.argv) != 4:
      print("Usage : pid_tuning_move.py robot_id max_linear_speed max_angular_speed")
    else:
      robot_id = sys.argv[1]
      robot = Robot('/' + robot_id + '/mode_auto', '/' + robot_id + '/auto_cmd_vel')
      
      max_linear_speed = float(sys.argv[2])
      max_angular_speed = float(sys.argv[3])
      
      slow_rotation = [Twist()]
      slow_rotation[0].angular.z = max_angular_speed/2

      slow_rotation_invert = [Twist()]
      slow_rotation_invert[0].angular.z = -max_angular_speed/2

      fast_rotation = [Twist()]
      fast_rotation[0].angular.z = max_angular_speed

      fast_rotation_invert = [Twist()]
      fast_rotation_invert[0].angular.z = -max_angular_speed

      slow_linear = [Twist()]
      slow_linear[0].linear.x = max_linear_speed/2

      slow_linear_invert = [Twist()]
      slow_linear_invert[0].linear.x = -max_linear_speed/2

      fast_linear = [Twist()]
      fast_linear[0].linear.x = max_linear_speed

      fast_linear_invert = [Twist()]
      fast_linear_invert[0].linear.x = -max_linear_speed

      # stop robot
      stop = [Twist()]

      robot.move(stop + 
        1 * stop +
        2 * slow_rotation + 
        2 * slow_rotation_invert +
        2 * fast_rotation_invert +
        2 * fast_rotation +
        1 * stop +
        2 * slow_linear +
        2 * slow_linear_invert +
        2 * fast_linear_invert +
        2 * fast_linear +
        1 * stop)

