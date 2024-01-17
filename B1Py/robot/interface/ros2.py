import struct
import threading
import time

import numpy as np
import numpy.linalg as LA
from scipy.spatial.transform import Rotation as R

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
from B1Py.msgs.ros2.unitree import HighCmd, HighState
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped

from B1Py.robot.model import PinRobot
from B1Py.joy import xKeySwitch, xRockerBtn


def ros2_init(args=None):
    rclpy.init(args=args)


def ros2_close():
    rclpy.shutdown()

class ROS2ExecutorManager:
    """A class to manage the ROS2 executor. It allows to add nodes and start the executor in a separate thread."""
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.executor_thread = None

    def add_node(self, node: Node):
        """Add a new node to the executor."""
        self.nodes.append(node)
        self.executor.add_node(node)

    def _run_executor(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.terminate()

    def start(self):
        """Start spinning the nodes in a separate thread."""
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.start()

    def terminate(self):
        """Terminate all nodes and shutdown rclpy."""
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        if self.executor_thread:
            self.executor_thread.join()


class B1HighLevelReal(Node):
    def __init__(
        self,
        test=False,
        vx_max=0.5,
        vy_max=0.4,
        ωz_max=0.5,
        node_name="b1py_highlevel_subscriber",
        cmd_topic_name="B1/high_cmd",
        state_topic_name="B1/high_state",
    ):
        self.node_name = node_name
        self.cmd_topic_name = cmd_topic_name
        self.state_topic_name = state_topic_name
        super().__init__(self.node_name)
        self.subscription = self.create_subscription(
            HighState, self.state_topic_name, self.new_state_callback, 1
        )
        self.cmd_publisher = self.create_publisher(HighCmd, self.cmd_topic_name, 1)
        self.cmd = HighCmd()
        self.cmd.mode = 0  # 0:idle, default stand 1:forced stand 2:walk continuously
        self.cmd.gait_type = 0
        self.cmd.speed_level = 0
        self.cmd.foot_raise_height = 0.0
        self.cmd.body_height = 0.0
        self.cmd.euler = [0.0, 0.0, 0.0]
        self.cmd.velocity = [0.0, 0.0]
        self.cmd.yaw_speed = 0.0
        self.cmd.reserve = 0
        self.ready = False

        # create pinocchio robot
        self.pin_robot = PinRobot()

        # for velocity clipping
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.P_v_max = np.diag([1 / self.vx_max**2, 1 / self.vy_max**2])
        self.ωz_max = ωz_max
        self.ωz_min = -ωz_max

        self.running = True

    def new_state_callback(self, msg):
        """
        Retrieve the state of the robot
        """
        self.state = msg

    def getIMU(self):
        accel = self.state.imu.accelerometer
        gyro = self.state.imu.gyroscope
        quat = self.state.imu.quaternion
        rpy = self.state.imu.rpy
        return accel, gyro, quat, rpy

    def getFootContacts(self):
        """Returns the foot contact states"""
        footContacts = self.state.foot_force_est
        return np.array(footContacts)

    def getJointStates(self):
        """Returns the joint angles (q) and velocities (dq) of the robot"""
        motorStates = self.state.motor_state
        _q, _dq = zip(
            *[(motorState.q, motorState.dq) for motorState in motorStates[:12]]
        )
        q, dq = np.array(_q), np.array(_dq)

        return q, dq

    def getRemoteState(self):
        """Returns the state of the remote control"""
        wirelessRemote = self.state.wireless_remote[:24]

        binary_data = bytes(wirelessRemote)

        format_str = "<2BH5f"
        data = struct.unpack(format_str, binary_data)

        head = list(data[:2])
        lx = data[3]
        rx = data[4]
        ry = data[5]
        L2 = data[6]
        ly = data[7]

        _btn = bin(data[2])[2:].zfill(16)
        btn = [int(char) for char in _btn]
        btn.reverse()

        keySwitch = xKeySwitch(*btn)
        rockerBtn = xRockerBtn(head, keySwitch, lx, rx, ry, L2, ly)

        return rockerBtn

    def getCommandFromRemote(self):
        """Do not use directly for control!!!"""
        rockerBtn = self.getRemoteState()

        lx = rockerBtn.lx
        ly = rockerBtn.ly
        rx = rockerBtn.rx

        v_x = ly * self.vx_max
        v_y = lx * self.vy_max
        ω = rx * self.ωz_max

        return v_x, v_y, ω

    def getBatteryState(self):
        """Returns the battery percentage of the robot"""
        batteryState = self.state.bms
        return batteryState.SOC

    def setCommands(self, v_x, v_y, ω_z, bodyHeight=0.0, footRaiseHeight=0.0, mode=2):
        assert mode in [0, 2]  # Only mode 2: walking and mode 0: idea is allowed
        self.cmd_watchdog_timer = time.time()
        self.cmd.mode = mode
        self.cmd.body_height = np.clip(bodyHeight, -0.15, 0.1)
        self.cmd.foot_raise_height = np.clip(footRaiseHeight, -0.1, 0.1)
        _v_x, _v_y, _ω_z = self.clip_velocity(v_x, v_y, ω_z)
        self.cmd.velocity = [_v_x, _v_y]
        self.cmd.yaw_speed = _ω_z
        # Publish the command as a ROS2 message
        self.cmd_publisher.publish(self.cmd)

    def close(self):
        self.running = False
        self.thread.join()
        self.destroy_node()

    def check_calf_collision(self, q):
        self.pin_robot.update(q)
        in_collision = self.pin_robot.check_calf_collision(q)
        return in_collision

    def clip_velocity(self, v_x, v_y, ω_z):
        _v = np.array([[v_x], [v_y]])
        _scale = np.sqrt(_v.T @ self.P_v_max @ _v)[0, 0]

        if _scale > 1.0:
            scale = 1.0 / _scale
        else:
            scale = 1.0

        return scale * v_x, scale * v_y, np.clip(ω_z, self.ωz_min, self.ωz_max)


class B1ExtrinsicsBroadcaster(Node):

    def __init__(self, robot_name='b1'):
        super().__init__(f'{robot_name}_tf2_broadcaster')

        self.frames = []
        # self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.body_frame_name = f'{robot_name}_base_link'

    def timer_callback(self):
        for frame in self.frames:
            self.broadcaster.sendTransform(frame)

    def registerFrame(self, sensor_name, body_T_sensor):
        frame = self.makeTransformStamped(self.body_frame_name, sensor_name, body_T_sensor)
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

class RealsenseExtrinsicsBroadcaster(Node):

    def __init__(self, camera_name,  oir1_T_oir2, oir1_T_ocolor, oir1_T_imu=None):
        super().__init__(f'{camera_name}_tf2_broadcaster')

        self.body_frame_name = f'{camera_name}_link'
        self.ir1_frame_name = f'{camera_name}_ir1_frame'
        self.ir2_frame_name = f'{camera_name}_ir2_frame'
        self.color_frame_name = f'{camera_name}_color_frame'
        if oir1_T_imu is not None:
            self.imu_frame_name = f'{camera_name}_imu_frame'
        self.ir1_optical_frame_name = f'{camera_name}_ir1_optical_frame'
        self.ir2_optical_frame_name = f'{camera_name}_ir2_optical_frame'
        self.color_optical_frame_name = f'{camera_name}_color_optical_frame'

        self.oir1_T_imu = oir1_T_imu
        self.oir1_T_oir2 = oir1_T_oir2
        self.oir1_T_ocolor = oir1_T_ocolor

        self.ros_T_optic = np.array([[0, 0, 1, 0],
                                [-1,0, 0, 0],
                                [0, -1, 0, 0],
                                [0, 0, 0, 1]]).astype(np.float64)

        # self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        body_T_ir1 = np.eye(4)
        body_T_ir2 = self.ros_T_optic @ self.oir1_T_oir2 @ np.linalg.inv(self.ros_T_optic)
        body_T_imu = self.ros_T_optic @ self.oir1_T_imu

        tf1 = self.makeTransformStamped(self.body_frame_name, self.ir1_frame_name, body_T_ir1)
        tf2 = self.makeTransformStamped(self.body_frame_name, self.ir2_frame_name, body_T_ir2)
        if self.oir1_T_imu is not None:
            tf3 = self.makeTransformStamped(self.body_frame_name, self.imu_frame_name, body_T_imu)
        tf4 = self.makeTransformStamped(self.ir1_frame_name, self.ir1_optical_frame_name, self.ros_T_optic)
        tf5 = self.makeTransformStamped(self.ir2_frame_name, self.ir2_optical_frame_name, self.ros_T_optic)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(tf1)
        self.broadcaster.sendTransform(tf2)
        if self.oir1_T_imu is not None:
            self.broadcaster.sendTransform(tf3)
        self.broadcaster.sendTransform(tf4)
        self.broadcaster.sendTransform(tf5)
        

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

class B1ExtrinsicsBroadcaster(Node):
    '''
    This class is used to broadcast the extrinsics of the robot as TF2 ROS2 static transforms.
    '''
    def __init__(self, robot_name='b1'):
        super().__init__(f'{robot_name}_tf2_broadcaster')

        self.frames = []
        # self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        # self.body_frame_name = f'{robot_name}_base_link'

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