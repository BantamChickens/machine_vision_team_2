import os
import time
import numpy as np
import cv2
import rclpy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from threading import Thread
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, Pose

# Points the computer to local fonts for the QT GUI that OpenCV will produce.
os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"


class optical_flow_estimate(Node):
    """
    ROS 2 node that estimates a Neato robot's position and heading using
    optical flow from a camera.

    The node processes raw camera images using an OpenCV optical flow model,
    identifies trackable features/keypoints, and calculates their movement
    between consecutive frames using the Lucas-Kanade optical flow algorithm.
    The resulting motion vectors are converted into meter and used
    to estimate the robot's movement.

    The estimated position and orientation are published as a PoseStamped
    message in the map coordinate frame.
    """

    def __init__(self, image_topic):
        """
        Initialize the optical flow localization node.

        Set up ROS 2 publishers and subscribers, initialize camera
        processing parameters, and start a separate thread for continuous
        optical flow processing.

        Args:
            image_topic: (str) ROS 2 topic to receive camera images from.
        """
        super().__init__("ball_tracker")
        self.cv_image = None  # the latest image from the camera
        self.bridge = CvBridge()  # used to convert ROS messages to OpenCV

        self.neato_position = np.zeros((2, 1))
        self.neato_heading = 0

        # distance from center of camera to the robots center im meters
        self.neato_center_to_camera = 0.305

        # subscribe to the camera image topic
        self.create_subscription(Image, image_topic, self.process_image, 10)

        # initialize ros2 params
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.image_pub = self.create_publisher(Image, "test_image", 10)

        self.raw_vectors_pub = self.create_publisher(Image, "raw_vectors", 10)
        self.pose_estimate_pub = self.create_publisher(
            PoseStamped, "pose_estimate", 10
        )
        self.image_output = True
        self.should_move = False  # the Neato should not be moving to start

        # params for Shi-Tomasi corner detection
        self.feature_params = dict(
            maxCorners=200, qualityLevel=0.2, minDistance=7, blockSize=7
        )

        # params for lucas kanade optical flow
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )

        # Start the optical flow processing thread
        print("Starting optical flow thread.")
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        print("test1")

    def update_pose(self):
        """
        Displays the (current) image stored in self.img in an OpenCV window.
        """
        cv2.imshow("frame", self.img)

    def process_image(self, msg=None):
        """
        Convert the ROS Image message into an OpenCV image, convert the
        image to grayscale, and apply histogram equalization to improve
        image contrast before optical flow processing.

        Args:
            msg: (sensor_msgs.msg.Image) optional Incoming
                ROS camera image message.
        """
        if not msg is None:
            """Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing"""
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grey_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
            self.cv_image = cv2.equalizeHist(grey_img)

    def loop_wrapper(self):
        """
        Call run_loop() repeatedly with a short delay between iterations.
        This function runs in a separate thread so image processing can
        continue independently of the ROS 2 spin loop.

        Sleep for regular pause in the image processing.
        """

        print("Executing Loop")
        while True:
            self.run_loop()
            time.sleep(0.01)

    def pixel_meter_conversion(self):
        """
        Convert optical flow pixel displacements into meters by
        using the camera's horizontal and vertical fields of view,
        along with its height above the ground, to estimate the
        physical distance represented by each pixel.

        Results are stored in self.corrected_vectors.

        Notes:
            This method uses a simplified camera geometry model and
            does not account for all perspective or depth variations.
        """
        self.corrected_vectors = np.zeros((self.raw_vectors.shape[0], 2))
        field_of_view_x = np.deg2rad(
            50.7
        )  # empirically measured with large error
        field_of_view_y = np.deg2rad(31.2)
        height = 0.114

        max_x_dist = 2 * math.sin(field_of_view_x / 2) * height  # in meters
        max_y_dist = 2 * math.sin(field_of_view_y / 2) * height

        pixels_x = self.last_frame.shape[1]
        pixels_y = self.last_frame.shape[0]
        dist_per_pixel_x = max_x_dist / pixels_x
        dist_per_pixel_y = max_y_dist / pixels_y

        for i in range(len(self.corrected_vectors)):
            new_dist_x = self.raw_vectors[i, 0, 1] * dist_per_pixel_x
            new_dist_y = self.raw_vectors[i, 1, 1] * dist_per_pixel_y

            # print(new_dist_x, new_dist_y)

            self.corrected_vectors[i, :] = [new_dist_x, new_dist_y]

    def optical_flow(self):
        """
        Calculate optical flow between consecutive camera frames.

        Use Shi-Tomasi corner detection to identify trackable features
        in the previous grayscale image. Apply the pyramidal
        Lucas-Kanade optical flow algorithm to track their movement
        in the current image.

        Store the detected feature displacements and their original
        positions in self.raw_vectors.

        If image output is enabled, draw the optical flow vectors
        onto the current frame and publish the visualization.

        Update self.last_frame after processing.

        Notes
        -----
        The raw_vectors array has dimensions:

            (number of tracked features, 2, 2)

        The second dimension represents the x and y coordinates.
        The third dimension stores the original feature position
        and its displacement.
        """

        color = np.array([[0, 0, 255] for i in range(300)])
        #  identify features to track in the previous frame
        p1 = cv2.goodFeaturesToTrack(
            self.last_frame, mask=None, **self.feature_params
        )

        # store the current frame
        self.current_frame = self.cv_image

        #  create mask for optical flow line visual overlay
        mask = np.zeros_like(cv2.cvtColor(self.last_frame, cv2.COLOR_GRAY2RGB))

        # find positions of points from last frame to current frame
        if not p1 is None:
            p2, st, err = cv2.calcOpticalFlowPyrLK(
                self.last_frame, self.current_frame, p1, None, **self.lk_params
            )

            #  Select good points
            if p2 is not None:
                good_new = p1[st == 1]
                good_old = p2[st == 1]

            #  setup array for all good raw vectors dimensions = (number of good vectors, dimensions(2), xy, uv)
            self.raw_vectors = np.zeros((np.shape(good_new)[0], 2, 2))
            if self.image_output:
                frame = None  # skip over adding the gray mask if there are no good points
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    #  gets new keypoint positions
                    a, b = new.ravel()
                    #  gets old keypoint positions
                    c, d = old.ravel()
                    #  saves vector
                    self.raw_vectors[i, :, 1] = [a - c, b - d]
                    self.raw_vectors[i, :, 0] = [c, d]
                    #  draws lines and dots on mask
                    mask = cv2.line(
                        mask,
                        (int(a), int(b)),
                        (int(c), int(d)),
                        color[i].tolist(),
                        2,
                    )
                    frame = cv2.circle(
                        mask, (int(a), int(b)), 5, color[i].tolist(), -1
                    )
                #  overlays mask and publishes optical flow example
                if frame is not None:
                    self.img = cv2.add(
                        frame,
                        cv2.cvtColor(self.current_frame, cv2.COLOR_GRAY2RGB),
                    )
                    pub_img = self.bridge.cv2_to_imgmsg(self.img)
                    self.image_pub.publish(pub_img)
                #  resets last fame
                self.last_frame = self.current_frame

    def find_mean_vector(self):
        """
        Compute the mean horizontal and vertical displacement from the
        scaled-corrected optical flow vectors.

        Determine the magnitude and direction of the average displacement
        vector and store the results as instance attributes.

        Results
        -------
        avg_vec : (list)
            Average displacement vector containing x and y components.
        avg_mag : (float)
            Magnitude of the average displacement vector.
        avg_ang : (float)
            Direction of the average displacement vector in radians.

        Also publish the corrected optical flow vectors as a ROS Image
        message on the raw_vectors topic.
        """
        angles = np.zeros((self.corrected_vectors.shape[0], 1))
        magnitudes = np.zeros((self.corrected_vectors.shape[0], 1))
        # for i in range(0, self.corrected_vectors.shape[0]):
        #     vec = self.corrected_vectors[i, :]
        #     angles[i] = np.arctan2(vec[0], vec[1])
        #     # print(vec)
        #     magnitudes[i] = np.sqrt(abs(vec[0]*vec[0]) + abs(vec[1]*vec[1]))
        self.avg_vec = [
            np.mean(self.corrected_vectors[:, 0]),
            np.mean(self.corrected_vectors[:, 1]),
        ]

        # median(angles) likely does not work because of wrap around negative vs positive angle equivalents
        self.avg_mag = np.linalg.norm(self.avg_vec)
        self.avg_ang = np.arctan2(self.avg_vec[1], self.avg_vec[0])
        # self.avg_vec = np.array([self.avg_mag * np.sin(self.avg_ang), self.avg_mag * np.cos(self.avg_ang)])
        # print(np.rad2deg(self.avg_ang))

        self.raw_vectors_pub.publish(
            self.bridge.cv2_to_imgmsg(self.corrected_vectors)
        )

    def modify_neato_localization(self):
        """
        Update the estimated Neato position and heading using optical flow.

        Interpret the average optical flow vector to determine whether
        the robot is moving forward, backward, or rotating.

        During rotation, estimate the change in heading using the
        displacement of the camera relative to the robot's center.

        During forward or backward movement, update the robot's
        estimated x and y position using the average displacement
        magnitude and its current heading.

        Updates
        -------
        neato_position: (numpy.ndarray) Estimated robot position in meters.
        neato_heading: (float) Estimated robot heading in radians.
        """

        # calculate the displacement in the neato's local direction
        vec = [
            np.cos(self.neato_heading) * self.avg_mag,
            np.sin(self.neato_heading) * self.avg_mag,
        ]

        # determine if the robot is rotating
        if not (3 * np.pi / 4 > np.abs(self.avg_ang) > np.pi / 4):
            x = self.avg_vec[0]
            y = self.avg_vec[1]
            length = self.neato_center_to_camera
            delta_heading = np.arctan2(x, length + y)
            self.neato_heading = self.neato_heading + delta_heading
            print("rotating")

        # determine if hte robot is moving forward
        elif 3 * np.pi / 4 > self.avg_ang > np.pi / 4:
            self.neato_position[0] += vec[0]  # x coord
            self.neato_position[1] += vec[1]  # y coord
            # print(vec)
            print("move forward")
            # print(self.avg_mag)

        # otherwise assume the robot is moving backward
        else:
            self.neato_position[0] -= vec[0]
            self.neato_position[1] -= vec[1]
            print("move backwards")

    def run_loop(self):
        """
        Execute one complete optical flow localization iteration.

        If a camera image is available:

        1. Save the current image as the previous frame.
        2. Calculate optical flow between consecutive frames.
        3. Convert pixel displacements into approximate meters.
        4. Calculate the average motion vector.
        5. Update the estimated robot position and heading.
        6. Convert the heading into a quaternion.
        7. Publish the estimated robot pose as a PoseStamped message.

        The published pose uses the map coordinate frame and includes
        a timestamp from the ROS 2 clock.
        """
        if not self.cv_image is None:
            #  save the initial camera frame
            self.last_frame = self.cv_image

            time.sleep(0.03)

            #  process optical flow and estimate robot's movement
            self.optical_flow()
            self.pixel_meter_conversion()
            self.find_mean_vector()
            self.modify_neato_localization()

            # convert estimated heading into a quaternion
            quat = R.from_euler(
                "z", self.neato_heading, degrees=False
            ).as_quat()

            # constructe the estimated robot pose
            msg = PoseStamped(
                pose=Pose(
                    position=Point(
                        x=float(self.neato_position[0].item()),
                        y=float(self.neato_position[1].item()),
                        z=0.0,
                    ),
                    orientation=Quaternion(
                        x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                    ),
                )
            )

            # set message timestampe and coordinate frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            # publish the estimated robot pose
            self.pose_estimate_pub.publish(msg)
            # print(msg)


def main(args=None):
    rclpy.init()
    n = optical_flow_estimate("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
