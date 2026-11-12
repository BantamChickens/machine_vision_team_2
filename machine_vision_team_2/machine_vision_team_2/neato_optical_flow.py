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

        self.neato_position = np.zeros((2, 1))
        self.neato_heading = 0

        self.neato_center_to_camera = 15 # center of camera is 15 inches from the center of the Neato

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, 'test_image', 10)

        self.raw_vectors_pub = self.create_publisher(Image, 'raw_vectors', 10)

        self.pose_estimate_pub = self.create_publisher(Pose, 'pose_estimate', 10)

        self.image_output = True

        self.should_move = False  # whether the Neato should be moving

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
        self.corrected_vectors = np.zeros((self.raw_vectors.shape[0], 2))
        field_of_view_x = 62.2
        field_of_view_y = 48.8
        height = 11.4

        max_x_dist = 2 * math.sin(field_of_view_x/2) * height
        max_y_dist = 2 * math.sin(field_of_view_y/2) * height

        pixels_x = self.last_frame.shape[0]
        pixels_y = self.last_frame.shape[1]
        dist_per_pixel_x =max_x_dist/pixels_x
        dist_per_pixel_y = max_y_dist/pixels_y
    
        for i in range(len(self.corrected_vectors)):
            #old_dist_x = old_vector_cloud[[i]][0] * dist_per_pixel_x # one vector in vector clouds x value and convert it to position in physical units
            #old_dist_y = old_vector_cloud[[i]][1] * dist_per_pixel_y
            new_dist_x = self.raw_vectors[i, 0, 1] * dist_per_pixel_x 
            new_dist_y = self.raw_vectors[i, 1, 1] * dist_per_pixel_y

            #print(new_dist_x, new_dist_y)

            #old_distance = math.sqt((old_dist_x **2)+(old_dist_y **2)) # postion of vector in first image
            distance = math.sqrt((new_dist_x **2)+(new_dist_y **2)) # position of vector in second image
            self.corrected_vectors[i, :] = [new_dist_x,new_dist_y]
            #self.corrected_vectors[i] = math.sqt(((self.corrected_vectors[[i]][0]) **2)+((self.corrected_vectors[[i]][1]) **2))


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
            vec = self.corrected_vectors[i, :]
            angels[i] = np.arctan2(vec[0], vec[1])
            #print(vec)
            magnitudes[i] = np.sqrt(abs(vec[0]*vec[0]) + abs(vec[1]*vec[1]))
        self.avg_mag = np.median(magnitudes)
        self.avg_ang = np.rad2deg(np.median(angels))
        self.avg_vec = np.array(self.avg_mag * np.cos(self.avg_ang), self.avg_mag * np.cos(self.avg_ang))

    def modify_neato_localization(self):
        # modifies the Neato position and heading based on the average vector from optical flow
        if 135 > self.avg_ang > 45 or 315 > self.avg_ang > 225: # if in the vertical movement range then modify the Neato position by the longitude vector
            vec = [np.cos(self.avg_ang) * self.avg_mag, np.sin(self.avg_ang) * self.avg_mag] # cos and sin already preserve the sign of the angle since arctan2 preserves exact angle
            self.neato_position[0] = self.neato_position[0] + vec[0] # yes i know there is a better way to add to vectors im just too tired to figure out the dimensions
            self.neato_position[1] = self.neato_position[1] + vec[1]
        else : # updates neato heading
            x = self.avg_vec[0]
            y = self.avg_vec[1]
            length = self.neato_center_to_camera
            delta_heading = np.arctan2(x, length+y)
            self.neato_heading = self.neato_heading - delta_heading
            
            



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