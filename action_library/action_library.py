import hello_helpers.hello_misc as hm
import multipoint_command as mc
import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import JointState

#import ik_solver

#Test
class ActionLibrary(mc.MultiPointCommand):

    def __init__(self):
        """
        Function that initializes the inhereted hm.HelloNode class.
        :param self: The self reference.
        """
        super().__init__()
        hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
        #self.ik = ik_solver.IKSolver()
        self.joint_states = None
        self.relevant_joints = ['wrist_extension', 'joint_lift', 'joint_wrist_yaw']


        #Subscribers and publishers
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)
        self.tag_publisher = rospy.Publisher(
            "tag_name", String, queue_size=1
        )

        self.tag_publisher.publish("None")
        
    
    def turnRight(self):
        "Rotates CCW 90 degrees"
        self.tag_publisher.publish("base_right")
        self.move_to_pose({'rotate_mobile_base':-np.pi/2})


    def turnLeft(self):
        """Rotaates CW 90 degrees"""
        self.move_to_pose({'joint_head_tilt':-1.8})
        self.tag_publisher.publish("base_left")
        self.move_to_pose({'rotate_mobile_base':np.pi/2})


    def toggleGripper(self, open):
        "Open/Close Gripper, [open] = false is close, reverse is true"
        magnitude = 0.2 #Roughly max opening for gripper, could go larger for closing
        if open:
            direction = -1
        else:
            direction = 1
        self.move_to_pose({'joint_gripper_finger_left':magnitude*direction})


    def moveLift(self, magnitude, direction):
        """
        Moves lift up(1) or down(-1) by magnitude, direction = 1 or -1

        Return: True if valid and successful, False otherwise
        """
        rospy.loginfo("test???")
        self.jointMovement(min= 0.2, max= 1, joint_name='joint_lift', magnitude=magnitude, direction=direction)


    def extend(self, magnitude, direction):
        """
        Extends/retracts based on direction(1/-1) and magnitude(int)
        
        Return: True if valid and successful, False otherwise
        """
        self.tag_publisher.publish("link_wrist_yaw_bottom")
        self.jointMovement(min=0, max=0.5, joint_name='wrist_extension',magnitude=magnitude, direction=direction)
        self.tag_publisher.publish("None")


#    def ik(self, goal):
#        #Raw ik move
#        """
#        Input: goal is a [x,y,z] position from base_link

#        Return: True if successful , false otherwise
#       """
#        joint_config = self.ik.goTo(goal)
#        self.move_to_pose(joint_config)
#        return True 
        

    def delivery(self):
        #Some trajectory command later
        self.issue_multipoint_command([[0.25, 0.93, 0, 0,0]], accelerations=[1,1,1,1])
        self.issue_multipoint_command([[0.25,0.9,0,0,-0.5], [0.25, 0.9, 0, 0.2,-0.4], [0.25, 0.9, 0 ,0.3,-0.3], [0.25, 0.9, 0 ,0,0], [0.25, 0.95, 0, 0.0,0],[0, 0.9, 2.5, 0,0]], velocities = [0.1,0.1,0.4,0.1, 0.3])
        self.turnRight()
        self.issue_multipoint_command([[0, 1.1, 2.5,0,0]], accelerations=[0.05,0.05,0.05,0.05,0])
        self.issue_multipoint_command([[0.4,1.1,0,0,0]], accelerations=[0.05,0.05,0.5,0.5,0], velocities=[0.5,1,0.4, 0,0])

    def retrieveFood(self):
        #
        pass 


    """
    
    def handleTransforms(self, tag_name):
        
        Gets transform vectors from aruco to base and aruco to camera, computes angle between them. Publishes transforms and camera angle needed
        to center on aruco tag 
        
        # Get transforms from the tag to the camera and base
        base_to_tag:TransformStamped = self.tf_buffer.lookup_transform('base_link', tag_name, rospy.Time(0))
        
        #Brian - There was a issue where it was more diffidult to get the angle while using the camera link as the source
        cam_to_tag:TransformStamped = self.tf_buffer.lookup_transform('camera_link', tag_name, rospy.Time(0))

        if (self.last_transform != base_to_tag.transform):
            self.count =0
            rospy.loginfo("Found Tag")
            # Get angle between the tag and the camera
            cam_hypotonuse = (cam_to_tag.transform.translation.x ** 2 + cam_to_tag.transform.translation.y ** 2) ** 0.5
            self.cam_to_tag_angle = -math.acos(base_to_tag.transform.translation.x/cam_hypotonuse)
            # Get angle between the tag and the base
            self.base_to_tag_angle = math.atan2(base_to_tag.transform.translation.y, base_to_tag.transform.translation.x)
            # Get distance between the tag and the base
            self.base_to_tag_distance = (base_to_tag.transform.translation.x ** 2 + base_to_tag.transform.translation.y ** 2) ** 0.5
            
            self.last_transform = base_to_tag.transform

    """



    def jointMovement(self, min:0, max:1, joint_name, magnitude, direction):
        """
        Moves joint by some magnitude based on direction

        Return: True if valid and successful, False otherwise
        """
        #Get joint position designated
        joints = self.print_states(self.relevant_joints)
        joint_position = joints[joint_name]

        #Check if mag + dir is possible, then attempt to execute
        offset = magnitude*direction
        if offset + joint_position > max or offset + joint_position < min:
            rospy.loginfo("Invalid parameters: Offset to large (>1) or too small (<0.2)")
            return False
        self.move_to_pose({joint_name: joint_position + magnitude*direction})
        return True


    #Move Later
    def callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg

    def print_states(self, joints):
        """
        print_states function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param joints: A list of string values of joint names.
        """
        joint_positions = {}
        for joint in joints:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions['wrist_extension'] = 4*self.joint_states.position[index]
                continue
            index = self.joint_states.name.index(joint)
            joint_positions[joint] = self.joint_states.position[index]
        return joint_positions
    

action = ActionLibrary()
rospy.sleep(1)
action.delivery()
try:
    rospy.spin()
except:
    rospy.loginfo("Bye")