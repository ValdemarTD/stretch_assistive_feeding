#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import ros_numpy as rnp
import time
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
from threading import Lock

class MouthDetection_withcrop():
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier('/home/hello-robot/catkin_ws/src/stretch_assistive_feeding/scripts/haarcascade_frontalface_default.xml')
        self.smile_cascade = cv2.CascadeClassifier('/home/hello-robot/catkin_ws/src/stretch_assistive_feeding/scripts/haarcascade_smile.xml')
        self.vid_subscriber = rospy.Subscriber('camera/color/image_raw', Image, self.camera_cb)
        self.depth_subscriber = rospy.Subscriber('camera/depth/color/points', PointCloud2, self.depth_cb)
        self.vid_publisher = rospy.Publisher('camera/color/image_detected', Image, queue_size=3)
        self.vid_bridge = CvBridge()
        self.mouth_publisher = rospy.Publisher('mouth_point', Point, queue_size=3)
        self.mouth_point = None
        self.mouth_lock = Lock()

        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(rospy.wait_for_message("camera/depth/camera_info", CameraInfo))

    def camera_cb(self, im):
        frame = cv2.rotate(cv2.cvtColor(self.vid_bridge.imgmsg_to_cv2(im), cv2.COLOR_BGR2RGB), cv2.ROTATE_90_CLOCKWISE)
        #cv_image = cv2.cvtColor(self.vid_bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)

        frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )    
        point_x = None
        point_y = None
        for (x, y, w, h) in faces:
            face_frame = frame[y:y+h, x:x+w]
            face_frame = cv2.resize(face_frame, dsize=(100,100), interpolation=cv2.INTER_AREA)
            cv2.rectangle(frame, (0,0), (10, 10), (255, 0, 0), 3)
            mouth_rects = self.smile_cascade.detectMultiScale(face_frame, 1.1, 11)
            #cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 3)
            for (x1,y1,w1,h1) in mouth_rects:
                if not y1 < h/3:
                    cv2.circle(frame, (int(x+x1+0.1*w1),int(y+y1+0.5*h1)), 3, (255,0,0), 3)
                    cv2.circle(frame, (int(x+x1+0.9*w1),int(y+y1+0.5*h1)), 3, (0,255,0), 3)
                    point_y = 720 - (x+x1+0.5*w1)
                    point_x = y+y1+0.5*h1
                break
        if not self.mouth_point == None:
            cv2.circle(frame, (self.mouth_point.x, self.mouth_point.y), 3, (128, 0, 255), 3)
        im_msg = self.vid_bridge.cv2_to_imgmsg(frame)
        self.vid_publisher.publish(im_msg)
        if point_x == None or point_y == None:
            return
        self.mouth_lock.acquire()
        self.mouth_point = Point()
        self.mouth_point.x = int(point_x)
        self.mouth_point.y = int(point_y)
        self.mouth_point.z = -1
        #self.mouth_publisher.publish(self.mouth_point)
        self.mouth_lock.release()

    def depth_cb(self, pc):
        #new_point = PointStamped()
        self.mouth_lock.acquire()
        if self.mouth_point == None:
            self.mouth_lock.release()
            return
        x = self.mouth_point.x
        y = self.mouth_point.y
        self.mouth_lock.release()
        unit_vec = self.cam_model.projectPixelTo3dRay((x,y))
        start = time.time()
        cloud_arr = rnp.point_cloud2.pointcloud2_to_xyz_array(pc)
        mid = time.time()
        lengths_arr = np.sqrt(np.einsum('...i,...i', cloud_arr, cloud_arr))
        uvec_arr = np.divide(cloud_arr, [lengths_arr, lengths_arr, lengths_arr])
        

        #uvec_arr = np.apply_along_axis(lambda lvar: lvar/(lvar**2).sum()**0.5, 1, cloud_arr)
        end = time.time()

        print(f"Cloud conversion took {mid - start}s")
        print(f"uvec array creation took {end - mid}s")
        print(uvec_arr)

        #print(uvec_arr)
        #print(np.apply_along_axis(np.linalg.norm, 0, cloud_arr))

        

if __name__ == '__main__':
    print("hi zac2: electric boogaloo")
    rospy.init_node("face", anonymous=True)
    md = MouthDetection_withcrop()
    print("Detection initialized")
    rospy.spin()


