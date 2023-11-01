import threading
import time

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class RosVelodyneListener:
    """
    Example Usage:

    rclpy.init(args=args)
    node = RosVelodyneListener()

    while node.points is None:
        continue

    pcd_o3d = o3d.geometry.PointCloud()
    pcd_o3d.points = o3d.utility.Vector3dVector(node.points)
    o3d.visualization.draw_geometries([pcd_o3d])

    node.stop()
    """

    def __init__(self, rate=10):
        """
        Initialize the velodyne listener.

        Parameters:
            rate (int, optional): Rate of point cloud update. Defaults to 10 Hz.
        """
        self.listener = PointCloudListener()
        self.rate = rate
        self.points = None

        self.stop_thread = False
        self.thread = threading.Thread(target=self.update)
        self.thread.start()

    def update(self):
        """
        Update the transformation matrix. This function runs in a separate thread
        and continuously listens for TF messages to update the transformation matrix.

        For the downsample and outlier removal portion of the code see:
        http://www.open3d.org/docs/latest/tutorial/Advanced/pointcloud_outlier_removal.html
        """
        pcd_o3d = o3d.geometry.PointCloud()

        while not self.stop_thread and rclpy.ok():
            try:
                rclpy.spin_once(self.listener)

                # downsample and remove outliers
                pcd_o3d.points = o3d.utility.Vector3dVector(self.listener.points)
                voxel_down_pcd = pcd_o3d.voxel_down_sample(voxel_size=0.02)
                pcd, _ = voxel_down_pcd.remove_statistical_outlier(20, 0.2)
                self.points = np.asarray(pcd.points)

            except:
                print("Cannot access point cloud data.")

            time.sleep(1 / self.rate)

    def stop(self):
        """
        Stops the thread that is updating the Lidar data.
        """
        self.stop_thread = True
        self.thread.join()
        self.listener.destroy_node()
        rclpy.shutdown()


class PointCloudListener(Node):
    def __init__(self):
        super().__init__("point_cloud_listener")
        self.subscription = self.create_subscription(
            PointCloud2, "/velodyne_points", self.listener_callback, 10
        )
        self.data = None

    def listener_callback(self, msg):
        # Read the x, y, z fields from the PointCloud2 message
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert the generator to a list, then to a numpy array
        self.points = np.array(list(gen))
