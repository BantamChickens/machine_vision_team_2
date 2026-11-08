import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt

class VectorVisiualizer(Node):
    """This is a message subscription node, which inherits from the rclpy Node class."""
    def __init__(self):
        self.bridge = CvBridge()  
        """Initializes the ReceiveMessageNode. No inputs."""
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(Image, 'raw_vectors', self.process_point, 10)



    def process_point(self, msg):
        self.raw_vector= self.bridge.imgmsg_to_cv2(msg) 
        print(self.raw_vector)
        for i in range(0, self.raw_vector.shape[0]):
            plt.quiver(self.raw_vector[i, 0, 0], self.raw_vector[i, 1, 0], self.raw_vector[i, 0, 1], self.raw_vector[i, 1, 1], color='b', units='xy', scale=1)

        # x-lim and y-lim
        plt.xlim(0, 700)
        plt.ylim(0, 700)

        # Show plot with grid
        plt.grid()
        plt.show()



def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)         # Initialize communication with ROS
    node = VectorVisiualizer()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

