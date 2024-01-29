import numpy as np
import yaml
from rclpy.node import Node
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import os
from b1py_calib.utils import *
import glob
from ament_index_python.packages import get_package_share_directory

params_root_dir = package_share_directory = get_package_share_directory('b1py_calib')
try:
    camera_intrinsics_dir = os.path.join(params_root_dir,'params/kalibr')
    camera_extrinsics_dir = os.path.join(params_root_dir,'params/simplehandeye')
    vicon2gt_results_dir = os.path.join(params_root_dir,'params/vicon2gt')   
    vicon2gt_result_file = os.path.join(vicon2gt_results_dir,os.listdir(vicon2gt_results_dir)[0])


except:
    print('Error: could not find calibration parameters. You need to store the calibration parameters in the following directories:'
    '\n\t- Kalibr: deploy/ros2_ws/src/b1py_calib/params/kalibr'
    '\n\t- SimpleHandEye: deploy/ros2_ws/src/b1py_calib/params/simplehandeye'
    '\n\t- vicon2gt: deploy/ros2_ws/src/b1py_calib/params/vicon2gt'
    '\nPlease refer to the documentation and make sure that you have the calibration parameters in the above directories and try again.')
    exit()


class B1ExtrinsicsBroadcaster(Node):

    def __init__(self, robot_name='b1'):
        super().__init__(f'{robot_name}_tf2_broadcaster')
        self.frames = []
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        info1 = KalibrExtractFrameExtrinsics(camera_intrinsics_dir)
        info2 = SimpleHandEyeExtractFrameExtrinsics(camera_extrinsics_dir)
        imu_T_vicon = Vicon2GtExtractParams(vicon2gt_result_file)
        # Instantiate a GTSAM factor graph manager pose graph manager
        manager = ExtrinsicCalibrationManager()
        # Add results from kalibr calibration procedure 
        for key in info1:
            parent = key.split(':')[0]
            child = key.split(':')[1]
            manager.add(parent, child, info1[key][0:3,0:3], info1[key][0:3, -1])
        # Add results from simplehandeye calibration procedure
        for key in info2:
            parent = key.split(':')[0]
            child = key.split(':')[1]
            manager.add(parent, child, info2[key][0:3,0:3], info2[key][0:3, -1])

        # Add results from vicon2gt calibration procedure
        manager.add('b1_imu_link', 'vicon/B1_BODY/B1_BODY', imu_T_vicon[0:3,0:3], imu_T_vicon[0:3, -1]) # Todo: make it configurable
        # Add aux TFs for the cameras
        ros_T_optic = np.array([[0, 0, 1, 0],
                        [-1,0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, 0, 1]]).astype(np.float64)
        keys = [key for key in manager.name_to_id.keys()]
        for key in keys:
            if key.endswith('optical_frame'):
                ros_frame = key.split('_optical_frame')[0]+'_frame'
                if key.endswith('infra1_optical_frame'):
                    body_frame = key.split('_infra1_optical_frame')[0]+'_link'
                elif key.endswith('infra2_optical_frame'):
                    body_frame = key.split('_infra2_optical_frame')[0]+'_link'
                elif key.endswith('color_optical_frame'):
                    body_frame = key.split('_color_optical_frame')[0]+'_link'
                elif key.endswith('depth_optical_frame'):
                    body_frame = key.split('_depth_optical_frame')[0]+'_link'
                elif key.endswith('imu_optical_frame'):
                    body_frame = key.split('_imu_optical_frame')[0]+'_link'
                if key.endswith('depth_optical_frame') or key.endswith('infra1_optical_frame'):
                    manager.add(body_frame, ros_frame, np.eye(3), np.zeros(3))
                manager.add(ros_frame, key, ros_T_optic[0:3,0:3], ros_T_optic[0:3, -1])
        # Get all transforms to formulate the tree
        all_tfs = manager.get_all('b1_imu_link')
        # Add the transforms to the tree
        for key in all_tfs:
            parent = key.split('_wrt_')[-1]
            child = key.split('_wrt_')[0]
            self.registerFrame(parent, child, all_tfs[key])
        self.registerFrame('b1_base_link', 'b1_imu_link', np.eye(4))
        T = np.eye(4)
        T[0,3]=0.2
        T[2,3]=0.3
        self.registerFrame('b1_imu_link', 'b1_rslidar', T)

    def timer_callback(self):
        for frame in self.frames:
            self.broadcaster.sendTransform(frame)

    def registerFrame(self, parent, child, body_T_sensor):
        frame = self.makeTransformStamped(parent, child, body_T_sensor)
        self.frames.append(frame)

    def makeTransformStamped(self, parent_frame, child_frame, T):
        rotation = R.from_matrix(T[:3, :3]).as_quat()
        translation = T[:3, 3]
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = parent_frame
        static_transform_stamped.child_frame_id = child_frame
        static_transform_stamped.transform.translation.x = translation[0]
        static_transform_stamped.transform.translation.y = translation[1]
        static_transform_stamped.transform.translation.z = translation[2]
        static_transform_stamped.transform.rotation.x = rotation[0]
        static_transform_stamped.transform.rotation.y = rotation[1]
        static_transform_stamped.transform.rotation.z = rotation[2]
        static_transform_stamped.transform.rotation.w = rotation[3]
        return static_transform_stamped

def main(args = None):
    rclpy.init(args=args)
    print('Starting the tf2 broadcaster node...')
    node = B1ExtrinsicsBroadcaster()
    print('Broadcasting the extrinsic transformations.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
