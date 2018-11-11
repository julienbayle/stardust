#!/usr/bin/env python

import roslib; roslib.load_manifest('sd_behavior')
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

mode_auto = False

cmd_vel_publisher = None
     
def mode_auto_callback(data):
    global mode_auto
    mode_auto = data.data

def teleop_cmd_vel_callback(data):
    global mode_auto, cmd_vel_publisher
    if (not mode_auto):
        cmd_vel_publisher.publish(data)
        
def auto_cmd_vel_callback(data):
    global mode_auto, cmd_vel_publisher
    if (mode_auto):
        cmd_vel_publisher.publish(data)

def main():
    rospy.init_node('sd_state_node')
    
    # Init publishers
    global cmd_vel_publisher
    cmd_vel_publisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)
    
    # Create subscribers
    rospy.Subscriber("teleop/cmd_vel", Twist, teleop_cmd_vel_callback)
    rospy.Subscriber("auto_cmd_vel", Twist, auto_cmd_vel_callback)
    rospy.Subscriber("mode_auto", Bool, mode_auto_callback)

    rospy.spin()


if __name__ == '__main__':
    main()