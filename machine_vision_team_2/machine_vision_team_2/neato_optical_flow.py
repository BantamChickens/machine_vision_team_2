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

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

        thread = Thread(target=self.loop_wrapper)
        thread.start()


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #print(self.cv_image)
        #print("procesed")


    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:       
            self.run_loop()
            time.sleep(5)
        cv2.destroyAllWindows()



    def optical_flow_test(self):
        color = np.random.randint(0, 255, (100, 3))
        #take to pictures to compare
        img_1 = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        p1 = cv2.goodFeaturesToTrack(img_1, mask = None, **self.feature_params)
        time.sleep(0.1)
        img_2 = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(img_1)
        #do optical flow analsys
        p2, st, err = cv2.calcOpticalFlowPyrLK(img_1, img_2, p1, None, **self.lk_params)

        # Select good points
        if p2 is not None:
            good_new = p1[st==1]
            good_old = p2[st==1]
            print("not none")

        # draw lines from old point to new
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv2.circle(img_2, (int(a), int(b)), 5, color[i].tolist(), -1)

        img = cv2.add(frame, mask)
        #cv2.imwrite("test.png", img);
        cv2.imshow('frame', img)

        print("hi")

    
    def run_loop(self):
        if not self.cv_image is None:
            self.optical_flow_test()


        

def main(args=None):
    rclpy.init()
    n = optical_flow_estimate("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()