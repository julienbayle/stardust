#!/usr/bin/env python

import roslib; roslib.load_manifest('sd_behavior')
import rospy
from tf.transformations import quaternion_from_euler

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sd_behavior_msgs.msg import Start

mode_auto = False

cmd_vel_publisher = None
initialpose_publisher = None

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

def start_callback(data):
    global initialpose_publisher

    team = 'yellow'
    if (data.team == Start.PURPLE):
        team = 'purple'

    initialpose = PoseWithCovarianceStamped()
    initialpose.header.frame_id = 'map'
    initialpose.header.stamp = rospy.Time.now()
    initialpose.pose.pose.position.x = rospy.get_param(team + '_x')
    initialpose.pose.pose.position.y = rospy.get_param(team + '_y')
    initialpose.pose.pose.position.z = 0.0
    quat = quaternion_from_euler(0.0, 0.0, rospy.get_param(team + '_yaw'))
    initialpose.pose.pose.orientation.x = quat[0]
    initialpose.pose.pose.orientation.y = quat[1]
    initialpose.pose.pose.orientation.z = quat[2]
    initialpose.pose.pose.orientation.w = quat[3]
    initialpose.pose.covariance[0] = 0.001
    initialpose.pose.covariance[7] = 0.001
    initialpose.pose.covariance[21] = 0.001

    initialpose_publisher.publish(initialpose)

def main():
    rospy.init_node('sd_state_node')
    
    # Init publishers
    global cmd_vel_publisher, initialpose_publisher
    cmd_vel_publisher = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)
    initialpose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    
    # Create subscribers
    rospy.Subscriber("teleop/cmd_vel", Twist, teleop_cmd_vel_callback)
    rospy.Subscriber("auto_cmd_vel", Twist, auto_cmd_vel_callback)
    rospy.Subscriber("mode_auto", Bool, mode_auto_callback)
    rospy.Subscriber("start", Start, start_callback)

    rospy.spin()


if __name__ == '__main__':
    main()