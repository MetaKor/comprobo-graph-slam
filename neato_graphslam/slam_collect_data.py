# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point

# Other imports
import graphslam.graph as gs
import numpy as np



class GraphSlam(Node):
    """ A class representing a ROS node that accumulates odometry and
        visual landmark data into a graphSLAM graph.
        Attributes:
            TODO
    """
    def __init__(self):
        super().__init__('graphslam')           # TODO does string input here need to be name of file????

        # Initialize graph object
        self.graph_vertices = []
        self.graph_edges = []
        #self.graph = gs.Graph([], [])

        # Set parameters
        self.odom_info_mat = np.array([])       # TODO define information matrix
        self.landmark_info_mat = np.array([])   # TODO define information matrix
        self.distance_thresh = 0.05             # [m] amount of linear movement before adding odometry link

        # Set up subscriptions
        self.create_subscription(Pose, 'odom', self.receive_odom, 10)
        self.create_subscription(Point, 'landmarks', self.receive_landmark, 10)

    def receive_odom(self):
        """
        """
        pass

    def receive_landmark(self):
        """
        """
        pass


def main(args=None):
    rclpy.init()
    n = GraphSlam()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
