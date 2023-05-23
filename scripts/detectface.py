import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MouthDetection():
    def init(self):
        # TODO: CHANGE THIS
        self.smile_cascade = cv2.CascadeClassifier('/home/ramya/my-workspace/src/stretch_assistive_feeding/scripts/haarcascade_smile.xml')
        self.vid_subscriber = rospy.Subscriber('camera/color/image_raw', Image, self.camera_cb)
        self.vid_publisher = rospy.Publisher('camera/color/image_raw', Image, queue_size=3)
        self.vid_bridge = CvBridge()

    def camera_cb(self, im):
        frame = cv2.rotate(self.vid_bridge.imgmsg_to_cv2(im), cv2.ROTATE_90_CLOCKWISE)
        #cv_image = cv2.cvtColor(self.vid_bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)

        frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        mouth_rects = self.smile_cascade.detectMultiScale(gray, 1.1, 11)
        for (x,y,w,h) in mouth_rects:
            if not h > 36:
                cv2.circle(frame, (int(x+0.1*w),y), 3, (255,0,0), 3)
                cv2.circle(frame, (int(x+0.9*w),y), 3, (0,255,0), 3)
            break
        im_msg = self.vid_bridge.cv2_to_imgmsg(frame)
        self.vid_publisher.publish(im_msg)


if __name__ == '__main__':
    print("hi zac")
    rospy.init_node("face", anonymous=True)
    rospy.spin()