import os
import time
import numpy as np
import cv2
import rclpy
import numpy as np
import matplotlib.pyplot as plt
import math

from threading import Thread
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray

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
        self.image_pub = self.create_publisher(Image, 'test_image', 10)

        self.raw_vectors_pub = self.create_publisher(Image, 'raw_vectors', 10)

        self.pose_estimate_pub = self.create_publisher(Pose, 'pose_estimate', 10)

        
        self.image_output = True

        self.should_move = False  # whether the Neato should be moving

        # TODO: initialize attributes for the center of the ball
        # TODO: initialize attributes for the color filtering thresholds

        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 200,
                       qualityLevel = 0.2,
                       minDistance = 7,
                       blockSize = 7 )
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

        thread = Thread(target=self.loop_wrapper)
        thread.start()


    def update_pose(self):
        cv2.imshow('frame', self.img)




    def process_image(self, msg = None):
        if not msg is None:
            """ Process image messages from ROS and stash them in an attribute
                called cv_image for subsequent processing """
            raw_img= self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grey_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
            self.cv_image = equ = cv2.equalizeHist(grey_img)


    def loop_wrapper(self):
        print("hi")
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:      
            self.run_loop()
            time.sleep(0.01)

    def paralax_corrector(self):
        self.corrected_vectors = self.raw_vectors


    def optical_flow(self):
        color = np.array([[0, 0, 255] for i in range(300)])
        #  find features to track in last image
        p1 = cv2.goodFeaturesToTrack(self.last_frame, mask = None, **self.feature_params)
        #  save current frame
        self.current_frame = self.cv_image
        #  create mask for flow line overlay
        mask = np.zeros_like(cv2.cvtColor(self.last_frame, cv2.COLOR_GRAY2RGB))
        # find positions of points from last frame to current frame
        if not p1 is None:
            p2, st, err = cv2.calcOpticalFlowPyrLK(self.last_frame, self.current_frame, p1, None, **self.lk_params)

            #  Select good points
            if p2 is not None:
                good_new = p1[st==1]
                good_old = p2[st==1]
            #  setup array for all good raw vectors dimensions = (number of good vectors, dimensions(2), xy, uv)
            self.raw_vectors = np.zeros((np.shape(good_new)[0], 2, 2))
            if self.image_output:
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    #  gets new keypoint positions
                    a, b = new.ravel()
                    #  gets old keypoint positions
                    c, d = old.ravel()
                    #  saves vector
                    self.raw_vectors[i, :, 1] = [a - c, b - d]
                    self.raw_vectors[i, :, 0] = [c, d]
                    #  draws lines and dots on mask
                    mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                    frame = cv2.circle(mask, (int(a), int(b)), 5, color[i].tolist(), -1)
                #  overlays mask and publishes optical flow example
                self.img = cv2.add(frame, cv2.cvtColor(self.current_frame, cv2.COLOR_GRAY2RGB))
                pub_img = self.bridge.cv2_to_imgmsg(self.img)
                self.image_pub.publish(pub_img)
                #if not raw_vectors.shape == (0, 2, 2):
                    #self.raw_vectors_pub.publish(self.bridge.cv2_to_imgmsg(raw_vectors))
                #  resets last fame
                self.last_frame = self.current_frame

    def find_mean_vector(self):
        angels = np.zeros((self.corrected_vectors.shape[0], 1))
        magnitudes = np.zeros((self.corrected_vectors.shape[0], 1))
        for i in range(0, self.corrected_vectors.shape[0]):
            vec = self.corrected_vectors[i, :, 1]
            angels[i] = np.arctan2(*vec.T[::-1])
            magnitudes[i] = np.sqrt(abs(vec[0]*vec[0]) + abs(vec[1]*vec[1]))
        avg_mag = np.mean(magnitudes)

        mdev_ang = np.median(np.abs(angels - np.median(angels)))
        s_ang = np.abs(angels - np.median(angels))/mdev_ang if mdev_ang else np.zeros(len(np.abs(angels - np.median(angels))))
        avg_ang = np.mean(angels[s_ang<2])

        mdev_mag = np.median(np.abs(magnitudes - np.median(magnitudes)))
        s_mag = np.abs(magnitudes - np.median(magnitudes))/mdev_mag if mdev_mag else np.zeros(len(np.abs(magnitudes - np.median(magnitudes))))
        avg_mag = np.mean(magnitudes[s_mag<2])

        self.avg_vec = np.array(avg_mag * np.cos(avg_ang), avg_mag * np.cos(avg_ang))

    def transform_to_neato_frame(self, vectors):
        transformed = vectors
        return transformed


    def run_loop(self):
        if not self.cv_image is None:
            #  takes initial picture
            self.last_frame = self.cv_image
            time.sleep(0.03)
            #  runs optical flow
            self.optical_flow()
            self.paralax_corrector()
            self.find_mean_vector()


        

def main(args=None):
    rclpy.init()
    n = optical_flow_estimate("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()