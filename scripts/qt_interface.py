#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout


#Based on the top answer to this question https://stackoverflow.com/questions/57204782/show-an-opencv-image-with-pyqt5
class DisplayImageWidget(QWidget):
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)

        self.button = QPushButton('Show picture')
        self.button.clicked.connect(self.show_image)
        self.image_frame = QLabel()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    @QtCore.pyqtSlot()
    def show_image(self, image):
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(image))


#Main window class
class MainWindow(QWidget):
    def __init__(self);
        self.vid_subscriber = rospy.Subscriber("camera/color/image_raw", Image, self.camera_cb)
        self.vid_bridge = CvBridge()

        self.vid_widget = DisplayImageWidget()

        self.layout = QVBoxLayout()
        self.buttons_widget = QWidget()
        self.buttons_layout = QHBoxLayout()

        self.

        self.layout.addWidget(self.vid_widget)

        self.setLayout(self.layout)

        

    def camera_cb(self, data):
        cv_image = cv2.cvtColor(self.vid_bridge.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
        qt_image = QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888)
        self.vid_widget.show_image(qt_image)


    def command_button_hit(self, )

if __name__=="__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    app.exec()