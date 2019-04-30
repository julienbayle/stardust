#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Robot:

    def __init__(self, auto_topic, cmd_vel_topic):
      self.auto_pub = rospy.Publisher(auto_topic, Bool, queue_size=10)
      self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
      
    def move(self, twist_array):

      self.auto_pub.publish(True)
      r = rospy.Rate(1)
      i = 0
      while not (rospy.is_shutdown() and i < len(twist_array) ):
        twist = twist_array[i]
        rospy.loginfo("Move %f-%f-%f" % (twist.linear.x, twist.linear.y, twist.angular.z))
        self.cmd_pub(twist)
        r.sleep()
        i = i+1

      self.auto_pub.publish(False)
        
if __name__ == '__main__':
  
    rospy.init_node('robot')
    robot = Robot('/r1/mode_auto', '/r1/auto_cmd_vel')
    
    # halt a turn per second
    t2 = Twist()
    turn.angular.z = 3.14

    # a quater a turn per second
    t4 = Twist()
    turn.angular.z = 3.14/2

    # stop robot
    stop = Twist()

    robot.move([stop, t4, t4, stop, t2, t2, stop])


    
