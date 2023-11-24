import threading
import time

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ROSPointCloudListener:
    def __init__(self, topic="/rslidar_points"):
        """
        Initialize the point cloud listener.

        Parameters:
            rate (int, optional): Rate of point cloud update. Defaults to 10 Hz.
        """
        rclpy.init()
        self.listener = PointCloudListener(topic)
        self.distance_threshold = 3.0

        self.R_IMULiDAR = np.array(
            [
                [0.999699, 0.0241473, 0.00425093],
                [-0.0240383, 0.999422, -0.0240584],
                [-0.00482942, 0.023949, 0.999702],
            ]
        )
        self.P_IMULIDAR = np.array([[0.2, 0.0, 0.3]])

        self.stop_thread = False
        self.thread = threading.Thread(target=self.update)
        self.thread.start()

    def update(self):
        rclpy.spin(self.listener)

    def process_points(self):
        self.listener.new_frame_flag = False

        # downsample and remove outliers
        pcd_o3d = o3d.geometry.PointCloud()
        pcd_o3d.points = o3d.utility.Vector3dVector(self.listener.points)
        voxel_down_pcd = pcd_o3d.voxel_down_sample(voxel_size=0.02)
        pcd, _ = voxel_down_pcd.remove_statistical_outlier(20, 0.2)

        # remove points that are too far away
        _points = np.asarray(pcd.points)
        squared_distances = np.sum(_points**2, axis=1)
        mask = squared_distances <= self.distance_threshold**2

        # _p = R @ p -> _p.T = p.T @ R.T
        self.points = _points[mask] @ self.R_IMULiDAR.T + self.P_IMULIDAR

        return _points[mask]

    def close(self):
        """
        Stops the thread that is updating the Lidar data.
        """
        self.stop_thread = True
        self.listener.destroy_node()
        rclpy.shutdown()
        self.thread.join()


class PointCloudListener(Node):
    def __init__(self, topic):
        super().__init__("point_cloud_listener")
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.listener_callback, 10
        )
        self.data = None
        self.new_frame_flag = False
        self.points = None

    def listener_callback(self, msg):
        # Read the x, y, z fields from the PointCloud2 message
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert the generator to a list, then to a numpy array
        self.points = np.array(list(gen))
        self.new_frame_flag = True
