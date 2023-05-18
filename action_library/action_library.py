import hello_helpers.misc as hm
import multipoint_command as mc

class ActionLibrary(mc.MultiPointCommand):

    def __init__(self):
        """
        Function that initializes the inhereted hm.HelloNode class.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)
        
    
    def turnRight():
        "Rotates CCW 90 degrees"
        pass

    def turnLeft():
        "Rotates CW 90 degrees"
        pass 

    def toggleGripper():
        "Open/Close Gripper"
        pass 

    def moveLift(self, magnitude, direction):
        #Add signs, magnitude later 
        self.move_to_pose({'joint_lift': weight*0.1})
        pass 

    def extend(self, magnitude, direction):
        self.move_to_pose({'wrist_extension': weight*0.1})

    def ik(self):
        #Raw ik move

    def delivery(self):
        #Some trajectory command later
        #  
        pass 

    def retrieveFood(self):
        #""
        pass 

    #Move Later
    def callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg

    def print_states(self):
        """
        print_states function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param joints: A list of string values of joint names.
        """
        joint_positions = []
        for joint in ['wrist_extension', 'joint_lift', 'joint_wrist_yaw']:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions.append(4*self.joint_states.position[index])
                continue
            index = self.joint_states.name.index(joint)
            joint_positions.append(self.joint_states.position[index])
        return joint_positions