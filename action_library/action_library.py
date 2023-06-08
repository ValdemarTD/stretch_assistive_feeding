import hello_helpers.hello_misc as hm
import multipoint_command as mc
import rospy
import numpy as np
import math
import math
import threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform
import tf2_ros
import time 
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
        self.follow_point = None

    

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #Initial Camera Data
        self.last_camera_angle = -math.pi / 4
        self.last_pan_camera_angle = 0

        
    
    def turnRight(self):
        "Rotates CCW 90 degrees"
        if self.handleTransforms("link_aruco_right_base"):
            self.move_to_pose({'joint_head_pan':self.base_to_tag_angle})
            self.move_to_pose({'joint_head_tilt':self.cam_to_tag_angle + np.pi/5})
            self.move_to_pose({'rotate_mobile_base':-np.pi/2})
        


    def turnLeft(self):
        """Rotaates CW 90 degrees"""
      
        if self.handleTransforms("link_aruco_left_base"):
            self.move_to_pose({'joint_head_pan':self.base_to_tag_angle})
            self.move_to_pose({'joint_head_tilt':self.cam_to_tag_angle})
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
    

    def deliveryMotion(self):
        "WIP Full grasp + delivery motion"
        self.moveWithCamera(target_point=[0, 1.0, np.pi/2],joints=['wrist_extension', 'joint_lift','joint_wrist_yaw'], target_accelerations=[0.05, 0.05, 0.5])
        self.moveWithCamera(target_point=[0.4, 0.4], joints=['wrist_extension','joint_wrist_yaw'], focus="link_aruco_top_wrist")



    def retrieveFood(self):
        self.moveWithCamera([0.25, 0.93, 0, 0,0], joints=['wrist_extension','joint_lift', 'joint_wrist_yaw', 'joint_wrist_roll', 'joint_wrist_pitch'], 
                            target_accelerations=[1,1,1,1,1])
        self.moveWithCamera(target_point= [0.9, 0,-0.5],joints=['joint_lift','joint_wrist_roll','joint_wrist_pitch'], target_velocities=[0.1, 0.1,0.3])
        self.moveWithCamera(target_point=[0.2,-0.4], joints=['joint_wrist_roll','joint_wrist_pitch'],  target_velocities=[0.1,0.3])
        self.moveWithCamera(target_point=[0.3,-0.3], joints=['joint_wrist_roll','joint_wrist_pitch'],  target_velocities=[0.1,0.3])
        self.moveWithCamera(target_point=[0,0], joints=['joint_wrist_roll','joint_wrist_pitch'],  target_velocities=[0.1,0.3])
        self.moveWithCamera(target_point=[0,np.pi/2], joints=["wrist_extension", "joint_wrist_yaw"], target_velocities=[0.1, 0.4])
        

    def randomMove(self):
        self.moveWithCamera([0,1, 0], joints=['wrist_extension', 'joint_lift','joint_wrist_yaw'])
        self.moveWithCamera([0.3,0.5, 1.5], joints=['wrist_extension', 'joint_lift', 'joint_wrist_yaw'])
        self.moveWithCamera([0.1,1.0, 2.3], joints=['wrist_extension', 'joint_lift', 'joint_wrist_yaw'])
        self.moveWithCamera([0,0.75, 0.8], joints=['wrist_extension', 'joint_lift', 'joint_wrist_yaw'])

    def jointsClose(self, joints, target_point):
        """
        Checks if joints are close enough/ if the movement was sucessful yet
        :param joints: A list of strings of joint names
        :param target_point list of target points in floats, in same order as joints
        """
        i=0
        truth = []
        joint_positions = self.print_states(joints=joints)
        print(joint_positions)
        for value in joint_positions.values():
            if np.isclose(value, target_point[i], atol=0.01):
                truth.append(True)
            else:
                truth.append(False)
            i+=1
        print(truth)
        return not (False in truth)


    def moveWithCamera(self, target_point, joints, target_velocities=None, target_accelerations=None, focus="utensil_end"):
        camera_joints = ['joint_head_pan', 'joint_head_tilt']
        combined_joints = camera_joints + joints

        #Change camera position until the intended target configuration is reached! 
        is_close = self.jointsClose(joints=joints, target_point=target_point) 
        while not is_close:
            if self.handleTransforms(focus):
                correction = np.pi/7
                pan_point = self.base_to_tag_angle
                tilt_point = self.cam_to_tag_angle + correction
                camera_target = [pan_point, tilt_point]
                final_target = camera_target + target_point

                camera_acceleration = [4,4]
                if target_accelerations is not None:
                    final_acceleration = camera_acceleration + target_accelerations
                else:
                    final_acceleration = None

                camera_velocities = [4,4]
                if target_velocities is not None:
                    final_velocity = camera_velocities + target_velocities
                else:
                    final_velocity = None
                #Send/update command
                self.issue_multipoint_command(shape=[final_target], 
                                              joints=combined_joints, accelerations=final_acceleration,
                                                velocities=final_velocity)
            #Update condition
            is_close = self.jointsClose(joints=joints, target_point=target_point)
            rospy.sleep(0.02)
        print("Ending While")

    def fullRetrieveAndDeliver(self):
        self.follow_point = "utensil_end"
        self.retrieveFood()
        self.turnRight()
        self.deliveryMotion()
        print("Completed")
        

    def mockIK(self):
        self.issue_multipoint_command([[0.4, 0.9, 0, 0,0],[0,0.95,2.5,0,0]])
        self.turnRight()
        self.issue_multipoint_command([[0.4,1.1,0,0,0]])


  
    
    def handleTransforms(self, tag_name):
        """
        Gets transform vectors from aruco to base and aruco to camera, computes angle between them. Publishes transforms and camera angle needed
        to center on aruco tag 
        """
        print(tag_name)
        try:
            # Get transforms from the tag to the camera and base
            base_to_tag:TransformStamped = self.tf_buffer.lookup_transform('base_link', tag_name, rospy.Time(0))
            
            #Brian - There was a issue where it was more diffidult to get the angle while using the camera link as the source
            cam_to_tag:TransformStamped = self.tf_buffer.lookup_transform( 'camera_link',tag_name, rospy.Time(0))
            #print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            #print("x",cam_to_tag.transform.translation.x, "y",cam_to_tag.transform.translation.y,"z", cam_to_tag.transform.translation.z)
            self.count =0
            # Get angle between the tag and the camera
            cam_hypotonuse = (cam_to_tag.transform.translation.x ** 2 + cam_to_tag.transform.translation.y ** 2) ** 0.5
       
            self.cam_to_tag_angle = -math.acos(base_to_tag.transform.translation.x/cam_hypotonuse)
   
            # Get angle between the tag and the base
            self.base_to_tag_angle = math.atan2(base_to_tag.transform.translation.y, base_to_tag.transform.translation.x)
           
            # Get distance between the tag and the base
            self.base_to_tag_distance = (base_to_tag.transform.translation.x ** 2 + base_to_tag.transform.translation.y ** 2) ** 0.5
            
            self.last_transform = base_to_tag.transform
          
            return True
        except:
            rospy.loginfo("Transformm not found?")
            return False




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

    def test_camera(self):
        self.single_joint('joint_head_tilt', [0,-1,0], accelerations=[4])
        self.single_joint()
        

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
        return: current positions of given joints
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
    

    ##CAMERA SPECIFIC METHODS
    def _pan_follower_callback(self, angle):
        #if abs(angle - self.last_pan_camera_angle) > 0.1:
        print(angle)
        new_pose = {"joint_head_pan": angle}
        #self.single_joint('joint_head_pan',points=[angle])
        self.move_to_pose(new_pose,return_before_done=True)
        self.last_pan_camera_angle = angle

    def _camera_following_callback(self, angle):
        if abs(angle - self.last_camera_angle) > 0.1:
            angle += np.pi/8
            new_pose = {"joint_head_tilt": angle}
            #self.single_joint('joint_head_tilt', points=[angle])
            self.move_to_pose(new_pose, return_before_done=True)
        self.last_camera_angle = angle
    
    def followTfCamera(self):
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            if self.follow_point is not None: 
                result = self.handleTransforms("utensil_end")
                if result:    
                    self._camera_following_callback(self.cam_to_tag_angle)
                    self._pan_follower_callback(self.base_to_tag_angle)
            else:
                rospy.loginfo("Follow Point Not Set")
            rate.sleep()

    def testing(self):
        # self.single_joint('joint_lift', [0.2,1,0.2,1,0.2,1,0.2,1])
        self.single_joint('joint_lift', [0.2,1])
        #rospy.sleep(2)
        #self.move_to_pose({"joint_lift":1})
    
    def testingtwo(self):
        self.single_joint('wrist_extension', [0.2,0.5,0.2,0.5,0.2,0.5,0.2,0.5])
        #rospy.sleep(2)

    def main(self):
        #Spits out camera movements
        if self.follow_point is not None: 
            result = self.handleTransforms(self.follow_point)
            if result:    
                self._camera_following_callback(self.cam_to_tag_angle)
                self._pan_follower_callback(self.base_to_tag_angle)
        
        

            

action = ActionLibrary()
#action_two =  ActionLibrary()
#rospy.sleep(1)



rospy.sleep(2)

action.randomMove()
#action.test_camera()
#action.move_to_pose({"wrist_extension":0.5}, return_before_done = True)
try:
    rospy.spin()
except:
    rospy.loginfo("Bye")