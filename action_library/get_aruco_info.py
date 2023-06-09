#! /usr/bin/env python3
import hello_helpers.hello_misc as hm

import rospy
import tf2_ros
from math import pi, sqrt, atan2

from std_srvs.srv import Trigger

from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
import math 
from std_msgs.msg import Float32, Int8, String
import numpy as np 


class ArucoNavigationNode():
    def __init__(self):

        #super().__init__()

        self.follow_tag_subscriber = rospy.Subscriber(
            "tag_name", String, self.follow_tag_callback
        )

        self.stored_tag_name = "link_wrist_yaw_bottom"


        #Controls how many duplicates it will tolerate before defaulting
        self.count = 0

        #Initial data for publishing
        self.cam_to_tag_angle = -math.pi/4
        self.base_to_tag_angle = 0
        self.base_to_tag_distance = 0

        self.cam_to_tag_angle_publisher = rospy.Publisher('cam_to_tag_angle', Float32, queue_size=1)
        self.base_to_tag_angle_publisher = rospy.Publisher('base_to_tag_angle', Float32, queue_size=1)
        self.base_to_tag_distance_publisher = rospy.Publisher('base_to_tag_distance', Float32, queue_size=1)

        self.last_transform = Transform()


    def follow_tag_callback(self, msg):
        self.stored_tag_name = msg.data

    def handleTransforms(self, tag_name):
            """
            Gets transform vectors from aruco to base and aruco to camera, computes angle between them. Publishes transforms and camera angle needed
            to center on aruco tag 
            """
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
            else:
                self.count +=1
                if self.count >= 100:
                    # The tf is old; ignore it
                    self.cam_to_tag_angle = -math.pi/4
                    self.base_to_tag_angle = 0
                    self.base_to_tag_distance = 0
            
                    

            # Publish all data
            self.cam_to_tag_angle_publisher.publish(self.cam_to_tag_angle)
            self.base_to_tag_angle_publisher.publish(self.base_to_tag_angle)
            self.base_to_tag_distance_publisher.publish(self.base_to_tag_distance)
            #rospy.loginfo(self.cam_to_tag_angle, self.base_to_tag_angle)

    def find_tag(self):
        '''  
        Sees if aruco tag is in frame and acts accordingly
        '''     
        #Check if tag is in view
        try:
            tag_name = self.stored_tag_name
            print(tag_name)
            self.handleTransforms(tag_name)
        #Tag not found
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
            rospy.loginfo("Problem finding tag")
            pass


        return None 

    def main(self):
        self.r = rospy.Rate(rospy.get_param('~rate', 10.0))


        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.find_tag()
            rate.sleep()
         
if __name__ == '__main__':
        
    try:
        rospy.init_node("Aruco_Info")
        node = ArucoNavigationNode()
        node.main()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
