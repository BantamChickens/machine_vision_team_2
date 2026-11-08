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


        self.image_output = True

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

        '''
        time.sleep(3)
        while(True):
            if not self.raw_vector_tails and self.raw_vector_tails is None:
                # Creating plot
                plt.quiver(self.raw_vector_tails[:,0,0], self.raw_vector_tails[:,0,1], self.raw_vector_heads[:,0,0], self.raw_vector_heads[:,0,1], color='b', units='xy', scale=1)
                plt.title('Single Vector')

                # x-lim and y-lim
                plt.xlim(-2, 5)
                plt.ylim(-2, 2.5)

                # Show plot with grid
                plt.grid()
                plt.show()
        '''


    def display_image(self):
        cv2.imshow('frame', self.img)


    def loop_2(self):
            time.sleep(3)
            print("thread")

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




    def optical_flow_test(self):
        color = np.random.randint(0, 255, (100, 3))
        #take to pictures to compare
        p1 = cv2.goodFeaturesToTrack(self.last_frame, mask = None, **self.feature_params)
        self.current_frame = self.cv_image
        #self.current_frame_blue = self.cv_image[:, :, 0]
        #print(self.current_frame.shape, self.current_frame_blue.shape)
        mask = np.zeros_like(self.last_frame)
        #do optical flow analsys
        if not p1 is None:
            p2, st, err = cv2.calcOpticalFlowPyrLK(self.last_frame, self.current_frame, p1, None, **self.lk_params)

            # Select good points
            if p2 is not None:
                good_new = p1[st==1]
                good_old = p2[st==1]
                #print("not none")
            
                raw_vectors = np.zeros((np.shape(good_new)[0], 2, 2))

            if self.image_output:
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    '''
                    a, b = new.ravel()
                    c, d = old.ravel()
                    print(c - a, d - b)
                    my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
                    point = Point(x = float(0), y = float(0), z = 0.0)
                    cross = np.cross([a, b, 0] , [c, d, 0])
                    dot = np.dot([a, b], [c, d])
                    quat = Quaternion(x = float(cross[0]), y = float(cross[0]), z = float(cross[2]), w = float(dot))
                    Poses.append(Pose(position = point, orientation = quat))'''
                    a, b = new.ravel()
                    c, d = old.ravel()
                    raw_vectors[i, :, 0] = np.array(a - c, b - d)
                    mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                    frame = cv2.circle(self.current_frame, (int(a), int(b)), 5, color[i].tolist(), -1)
                #posearray = PoseArray(poses = Poses, header = my_header, check_fields = 0)
                #print("hi")
                #self.unscaled_vectors.publish(posearray)
                self.img = cv2.add(self.current_frame, mask)
                pub_img = self.bridge.cv2_to_imgmsg(self.img)
                self.image_pub.publish(pub_img)
                if not raw_vectors.shape == (0, 2, 2):
                    self.raw_vectors_pub.publish(self.bridge.cv2_to_imgmsg(raw_vectors))
                self.last_frame = self.current_frame


    
    def run_loop(self):
        if not self.cv_image is None:
            self.last_frame = self.cv_image
            time.sleep(0.03)
            self.optical_flow_test()


        

def main(args=None):
    rclpy.init()
    n = optical_flow_estimate("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()