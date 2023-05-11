#!/usr/bin/env python
import rospy
import sys
import os
import cv2
import logging
import time
import shapely
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QMainWindow


class NavigationController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.go_forwards
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.go_backwards
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.turn_left
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.turn_right
            }
        ]
        self.parent = parent
        self.move_publisher = rospy.Publisher("/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=5)

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def go_forwards(self):
        msg = Twist()
        msg.linear.x = 2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)
        print("Go forwards")

    def go_backwards(self):
        msg = Twist()
        msg.linear.x = -2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)
        print("Go backwards")
    
    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        print("Turn left")

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = -2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        print("Turn right")

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")



#Based on the top answer to this question https://stackoverflow.com/questions/57204782/show-an-opencv-image-with-pyqt5
class DisplayImageWidget(QWidget):
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)


        #self.button = QPushButton('Show picture')
        #self.button.clicked.connect(self.show_image)
        self.image_frame = QLabel()
        self.image_frame.setFixedSize(480, 640)

        self.nav_controller = NavigationController(parent=self)

        self.mode = "camera"

        self.available_modes = {
            "camera" : {
                "show_function" : self.only_show_image,
                "controller" : None
            },
            "navigation" : {
                "show_function" : self.show_navigation,
                "controller" : self.nav_controller
            }
        }

        self.image_frame.mousePressEvent = self.click_event
        self.image_frame.enterEvent = self.mouseover_event

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    def set_mode(self, mode):
        if mode in self.available_modes.keys():
            self.mode = mode
            logging.debug(f"Switching to mode {mode}")
        else:
            self.mode = "camera"
            logging.warning(f"Interface has no mode \"{mode}\"")

    def mouseover_event(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].mouseover_event(event)

    def click_event(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].click_event(event)

    def only_show_image(self, cv_image):
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def show_navigation(self, cv_image):
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def show_image_by_mode(self, cv_image):
        self.available_modes[self.mode]["show_function"](cv_image)

    @QtCore.pyqtSlot()
    def show_image(self, image):
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(image))


#Main window class
class MainWindow(QMainWindow):
    def __init__(self):
        super(QWidget, self).__init__()
        self.vid_subscriber = rospy.Subscriber("camera/color/image_raw", Image, self.camera_cb)
        self.vid_bridge = CvBridge()

        self.vid_widget = DisplayImageWidget(parent=self)

        self.main_layout = QVBoxLayout()

        self.top_widget = QWidget()
        self.top_layout = QHBoxLayout()

        self.buttons_widget = QWidget()
        self.buttons_layout = QHBoxLayout()

        self.camera_mode_button = QPushButton(text="Camera", parent=self.buttons_widget)
        self.camera_mode_button.setCheckable(True)
        self.camera_mode_button.setAutoExclusive(True)
        self.camera_mode_button.setChecked(True)
        self.camera_mode_button.setFixedSize(100, 100)
        self.camera_mode_button.clicked.connect(lambda: self.change_video_mode("camera"))

        self.navigation_mode_button = QPushButton(text="Navigation", parent=self.buttons_widget)
        self.navigation_mode_button.setCheckable(True)
        self.navigation_mode_button.setAutoExclusive(True)
        self.navigation_mode_button.setFixedSize(100, 100)
        self.navigation_mode_button.clicked.connect(lambda: self.change_video_mode("navigation"))
        
        self.buttons_layout.addWidget(self.camera_mode_button)
        self.buttons_layout.addWidget(self.navigation_mode_button)

        self.buttons_widget.setLayout(self.buttons_layout)

        self.top_layout.addWidget(self.buttons_widget)

        self.top_widget.setLayout(self.top_layout)

        self.main_layout.addWidget(self.top_widget)

        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        self.actionlib_widget = QWidget()
        self.actionlib_layout = QVBoxLayout()
        self.actionlib_widget.setLayout(self.actionlib_layout)

        self.middle_layout.addWidget(self.vid_widget)
        self.middle_layout.addWidget(self.actionlib_widget)
        self.middle_widget.setLayout(self.middle_layout)

        self.main_layout.addWidget(self.middle_widget)


        self.status_layout = QHBoxLayout()
        self.status_widget = QWidget()
        self.status_widget.setLayout(self.status_layout)

        self.main_layout.addWidget(self.status_widget)

        self.main_widget = QWidget()
        self.main_widget.setLayout(self.main_layout)

        self.setCentralWidget(self.main_widget)

        

    def camera_cb(self, data):
        cv_image = cv2.rotate(cv2.cvtColor(self.vid_bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB), cv2.ROTATE_90_CLOCKWISE)
        self.vid_widget.show_image_by_mode(cv_image)


    def change_video_mode(self, mode):
        self.vid_widget.set_mode(mode)

if __name__=="__main__":
    logging.basicConfig(format='%(levelname)s %(asctime)s: %(message)s',filename=f"{os.path.dirname(os.path.abspath(__file__))}/logs/qt_interface_{time.strftime('%y_%m_%d:%H_%M_%S', time.localtime(time.time()))}.log", level=logging.INFO, datefmt="%y-%m-%d:%h-%m-%s")
    logging.info("Starting application")

    rospy.init_node("qt_interface", anonymous=True)

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    logging.info("Application started")

    app.exec()