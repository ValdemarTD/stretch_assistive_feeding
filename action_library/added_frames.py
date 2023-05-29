#!/usr/bin/env python3

import rospy
import tf.transformations
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np

class FixedFrameBroadcaster():
    """
    This node publishes three child static frames in reference to their parent frames as below:
    parent -> link_mast            child -> fk_link_mast
    parent -> link_lift            child -> fk_link_lift
    parent -> link_wrist_yaw       child -> fk_link_wrist_yaw
    """
    def __init__(self):
        """
        A function that creates a broadcast node and publishes three new transform
        frames.
        :param self: The self reference.
        """
        self.br = StaticTransformBroadcaster()

        self.lift = TransformStamped()
        self.lift.header.stamp = rospy.Time.now()
        self.lift.header.frame_id = 'link_wrist_roll'
        self.lift.child_frame_id = 'utensil_end'
        self.lift.transform.translation.x = 0
        self.lift.transform.translation.y = -0.08
        self.lift.transform.translation.z = 0.165
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.lift.transform.rotation.x = q[0]
        self.lift.transform.rotation.y = q[1]
        self.lift.transform.rotation.z = q[2]
        self.lift.transform.rotation.w = q[3]


        self.br.sendTransform([self.lift])

        rospy.loginfo('Publishing TF frames. Use RViz to visualize')

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    FixedFrameBroadcaster()
    rospy.spin()
