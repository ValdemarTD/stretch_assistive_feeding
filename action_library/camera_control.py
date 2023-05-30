#! /usr/bin/env python3
import rospy
from std_srvs.srv import Trigger
import hello_helpers.hello_misc as hm

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint
import math
from std_msgs.msg import Float32, Int8, String


class JointControl(hm.HelloNode):
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """

    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(
            self, "camera_follower", "camera_follower", wait_for_first_pointcloud=False
        )
        self.joint_state = None
        self.last_camera_angle = -math.pi / 4
        self.last_pan_camera_angle = 0
        self.stored_tag_name = "None"

        rospy.loginfo(
            "{0}: Made contact with trajectory server".format(self.__class__.__name__)
        )
        self.camera_angle_subscriber = rospy.Subscriber(
            "cam_to_tag_angle", Float32, self.camera_following_callback
        )
        self.base_to_tag_angle_subscriber = rospy.Subscriber(
            "base_to_tag_angle", Float32, self.pan_follower_callback
        )
        self.follow_tag_subscriber = rospy.Subscriber(
            "tag_name", String, self.follow_tag_callback
        )

    def follow_tag_callback(self, msg):
        self.stored_tag_name = msg.data

    def pan_follower_callback(self, msg):
        if self.stored_tag_name != "None":
            angle = 1 * msg.data
            if abs(angle - self.last_camera_angle) > 0.7:
                rospy.loginfo("Pan to %s radians", angle)
                new_pose = {"joint_head_pan": angle}
                self.move_to_pose(new_pose)
            self.last_pan_camera_angle = angle

    def camera_following_callback(self, msg):
        if self.stored_tag_name != "None":
            angle = 1 * msg.data
            if abs(angle - self.last_camera_angle) > 0.1:
                rospy.loginfo("Tilt to %s radians", angle)
                new_pose = {"joint_head_tilt": angle}
                self.move_to_pose(new_pose)
            self.last_camera_angle = angle


    def joint_states_callback(self, joint_state):
        """
        Callback for the /stretch/joint_states topic to store the current joint states for use within the class
        """
        self.joint_state = joint_state
        

    def main(self):
        """
        Function that initiates the multipoint_command function.
        :param self: The self reference.
        """
        # Setting up relevant nodes and subscribers
        rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_states_callback
        )

        rospy.loginfo("completed intial setup...")
        min_pan = -4
        max_pan = 1.3
        delta = (max_pan - min_pan) / 5
        rate = rospy.Rate(10)
        #if self.stored_tag_name is not 'None':
        """
        For sweeping: 
        joint_index = self.joint_state.name.index("joint_head_pan")
        joint_value = self.joint_state.position[joint_index]
        rospy.loginfo("Delta %s", delta)
        if (delta + joint_value) > 1.3 or (delta + joint_value) < -4:
            delta = -delta
        command = {"joint": "joint_head_pan", "delta": delta}
        self.send_command(command)
        """
            


if __name__ == "__main__":
    #rospy.init_node("test")
    node = JointControl()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("interrupt received, so shutting down")
