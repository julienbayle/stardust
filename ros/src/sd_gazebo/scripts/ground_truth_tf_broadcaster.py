#!/usr/bin/env python  
import roslib
roslib.load_manifest('sd_gazebo')
import rospy

import tf
import nav_msgs.msg

def handle_ground_truth(msg, (tf_prefix, target_frame)):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     '%s/%s' % (tf_prefix, msg.child_frame_id),
                     target_frame)

if __name__ == '__main__':
    rospy.init_node('ground_truth_tf_broadcaster')
    tf_prefix = rospy.get_param('~tf_prefix')
    target_frame = rospy.get_param('~target_frame', "world")
    rospy.Subscriber('ground_truth',
                     nav_msgs.msg.Odometry,
                     handle_ground_truth,
                     (tf_prefix, target_frame))
    rospy.spin()