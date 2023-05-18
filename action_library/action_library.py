import hello_helpers.misc as hm
import multipoint_command as mc
import rospy


import ik_solver

#Test
class ActionLibrary(mc.MultiPointCommand):

    def __init__(self):
        """
        Function that initializes the inhereted hm.HelloNode class.
        :param self: The self reference.
        """
        self.ik = ik_solver.IKSolver()
        hm.HelloNode.__init__(self)
        self.relevant_joints = ['wrist_extension', 'joint_lift', 'joint_wrist_yaw']
        
    
    def turnRight():
        "Rotates CCW 90 degrees"
        pass

    def turnLeft():
        "Rotates CW 90 degrees"
        pass 

    def toggleGripper(self, open):
        "Open/Close Gripper"
        magnitude = 0.2
        if open:
            direction = -1
        else:
            direction = 1
        self.move_to_pose({'joint_gripper_finger_left':magnitude*direction})



    def moveLift(self, magnitude, direction):
        """
        Moves lift up(1) or down(-1) by magnitude

        Return: True if valid and successful, False otherwise
        """
        self.jointMovement(min= 0.2, max= 1, joint_name='joint_lift', magnitude=magnitude, direction=direction)

    def extend(self, magnitude, direction):
        """
        Extends/retracts based on direction(1/-1) and magnitude(int)
        
        Return: True if valid and successful, False otherwise
        """
        self.jointMovement(min=0, max=0.5, joint_name='wrist_extension',magnitude=magnitude, direction=direction)

    def ik(self, goal):
        #Raw ik move
        """
        Input: goal is a [x,y,z] position from base_link
        """
        
        pass

    def delivery(self):
        #Some trajectory command later
        pass 

    def retrieveFood(self):
        #
        pass 




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
        self.move_to_pose({'joint_lift': joint_position + magnitude*direction})
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