import struct
import threading
import time

import numpy as np
import numpy.linalg as LA

from B1Py.pinocchio import PinRobot
from B1Py.utils import xKeySwitch, xRockerBtn
import rclpy
from rclpy.node import Node
from unitree_msgs.msg import HighState, HighCmd
from rclpy.qos import QoSProfile
import threading
import time

def ros2_init(args=None):
    rclpy.init(args=args)

def ros2_close():
    rclpy.shutdown()


class B1HighLevelReal(Node):
    def __init__(
        self, test=False, vx_max=0.5, vy_max=0.4, ωz_max=0.5,
        node_name= 'b1py_highlevel_subscriber', 
        cmd_topic_name= 'B1/high_cmd',
        state_topic_name= 'B1/high_state'):  
        self.node_name = node_name
        self.cmd_topic_name = cmd_topic_name
        self.state_topic_name = state_topic_name
        super().__init__(self.node_name)
        self.subscription = self.create_subscription(
            HighState,
            self.state_topic_name,
            self.new_state_callback,
            1)
        self.cmd_publisher = self.create_publisher(HighCmd, self.cmd_topic_name, 1)
        self.cmd = HighCmd()
        self.cmd.mode = 0  # 0:idle, default stand 1:forced stand 2:walk continuously
        self.cmd.gaitType = 0
        self.cmd.speedLevel = 0
        self.cmd.footRaiseHeight = 0
        self.cmd.bodyHeight = 0
        self.cmd.euler = [0, 0, 0]
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = 0.0
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

        # if not test:
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        print(f'{self.node_name} ROS2 interface is running...')
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.2)

    def new_state_callback(self, msg):
        """
        Retrieve the state of the robot
        """
        self.state = msg.copy()

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
