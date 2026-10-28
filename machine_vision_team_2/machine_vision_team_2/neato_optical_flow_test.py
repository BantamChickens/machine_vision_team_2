import os
import time
import numpy as np
import cv2
import rclpy

from threading import Thread
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Points the computer to local fonts for the QT GUI that OpenCV will produce.
os.environ['QT_QPA_FONTDIR'] = '/usr/share/fonts/truetype/dejavu'

class optical_flow_estimate(Node):

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        super().__init__('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.should_move = False  # whether the Neato should be moving

        # TODO: initialize attributes for the center of the ball
        # TODO: initialize attributes for the color filtering thresholds

        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def optical_flow(self):
        pass

    
    def run_loop(self):
        pass

def main(args=None):
    rclpy.init()
    n = optical_flow("camera/image")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()