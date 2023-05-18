#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import time

class CameraHandler:
  def __init__(self):
    self.subscriber = rospy.Subscriber("camera/color/image_raw", Image, self.camera_cb)
    self.bridge = CvBridge()
    self.last_image = None
    
  def camera_cb(self, data):
    cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    _, buffer = cv2.imencode('.jpg', cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE))
    self.last_image = buffer.tobytes()

  def get_time(self):
    yield str(time.time())

  def get_last_image(self):
    while self.last_image == None:
      time.sleep(0.1)
    while True:
      yield (b'--frame\r\n' + b'Content-Type: image/jpeg\r\n\r\n' + self.last_image + b'\r\n')