import rospy
from std_msgs.msg import String


class CommandHandler():
    def __init__(self):
        self.publisher = rospy.Publisher("feeding_commands", String)
    
    def send_command(self, command):
        new_msg = String()
        new_msg.data = command
        self.publisher.publish(new_msg)