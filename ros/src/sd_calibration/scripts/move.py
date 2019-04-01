#!/usr/bin/env python

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
    robot = Robot('/r1/mode_auto', '/r1/auto_cmd_vel')
    
    t1 = [Twist()]
    t1[0].angular.z = 2

    t2 = [Twist()]
    t2[0].angular.z = -2
    t3 = [Twist()]
    t3[0].angular.z = 1

    t4 = [Twist()]
    t4[0].angular.z = -1


    # stop robot
    stop = [Twist()]

    robot.move(stop + 3 * t1 + 3 * t3 + 3 * t2 + 3*t4 + stop)

