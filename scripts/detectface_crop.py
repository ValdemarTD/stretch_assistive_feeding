#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class MouthDetection_withcrop():
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier('/home/hello-robot/catkin_ws/src/stretch_assistive_feeding/scripts/haarcascade_frontalface_default.xml')
        self.smile_cascade = cv2.CascadeClassifier('/home/hello-robot/catkin_ws/src/stretch_assistive_feeding/scripts/haarcascade_smile.xml')
        self.vid_subscriber = rospy.Subscriber('camera/color/image_raw', Image, self.camera_cb)
        self.vid_publisher = rospy.Publisher('camera/color/image_detected', Image, queue_size=3)
        self.vid_bridge = CvBridge()
        self.mouth_publisher = rospy.Publisher('mouth_point', Point, queue_size=3)

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

            mouth_rects = self.smile_cascade.detectMultiScale(face_frame, 1.1, 11)
            
            for (x1,y1,w1,h1) in mouth_rects:
                if not h1 > 36:
                    cv2.circle(frame, (int(x+x1+0.1*w1),int(y+y1+0.5*h1)), 3, (255,0,0), 3)
                    cv2.circle(frame, (int(x+x1+0.9*w1),int(y+y1+0.5*h1)), 3, (0,255,0), 3)
                    point_x = x+x1+0.5*w1
                    point_y = y+y1+0.5*h1
                break
        im_msg = self.vid_bridge.cv2_to_imgmsg(frame)
        self.vid_publisher.publish(im_msg)
        if point_x == None or point_y == None:
            return
        mouth_pt = Point()
        mouth_pt.x = int(x1+0.5*w1)
        mouth_pt.y = y1
        self.mouth_publisher.publish(mouth_pt)

if __name__ == '__main__':
    print("hi zac2: electric boogaloo")
    rospy.init_node("face", anonymous=True)
    md = MouthDetection_withcrop()
    print("Detection initialized")
    rospy.spin()


