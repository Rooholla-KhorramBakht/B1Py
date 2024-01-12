import struct
import threading
import time

import numpy as np
import numpy.linalg as LA
import robot_interface as sdk

from B1Py.robot.model import PinRobot
from B1Py.joy import xKeySwitch, xRockerBtn


class B1HighLevelReal:
    def __init__(
        self, loop_frequency=100, test=False, vx_max=0.5, vy_max=0.4, ωz_max=0.5
    ):
        self.loop_frequency = loop_frequency
        HIGHLEVEL = 0xEE
        self.udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.220", 8082)
        self.cmd = sdk.HighCmd()
        self.state = sdk.HighState()
        self.udp.InitCmdData(self.cmd)
        self.cmd.mode = (
            0  # 0:idle, default stand      1:forced stand     2:walk continuously
        )
        self.cmd.gaitType = 0
        self.cmd.speedLevel = 0
        self.cmd.footRaiseHeight = 0
        self.cmd.bodyHeight = 0
        self.cmd.euler = [0, 0, 0]
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = 0.0
        self.cmd.reserve = 0
        self.ready = False
        self.cmd_watchdog_timer = time.time()

        # create pinocchio robot
        self.pin_robot = PinRobot()

        # for velocity clipping
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.P_v_max = np.diag([1 / self.vx_max**2, 1 / self.vy_max**2])
        self.ωz_max = ωz_max
        self.ωz_min = -ωz_max

        if not test:
            self.running = True
            self.loop_thread = threading.Thread(target=self.loop_fn)
            self.loop_thread.start()

    def loop_fn(self):
        while self.running:
            self.udp.Recv()
            self.udp.GetRecv(self.state)

            if time.time() - self.cmd_watchdog_timer > 0.2:
                self.cmd.mode = 0
                self.cmd.euler = [0, 0, 0]
                self.cmd.velocity = [0, 0]
                self.cmd.yawSpeed = 0.0
            self.udp.SetSend(self.cmd)
            self.udp.Send()
            time.sleep(1 / self.loop_frequency)
            # Communication link is not ready at the beginning of transaction
            # To know it is ready make sure the IMU data makes sense
            if not self.ready and LA.norm(self.getIMU()[0]) > 5:
                self.ready = True

    def retrieve_state(self):
        """
        Retrieve the state of the robot (only use in test mode)
        """
        self.udp.Recv()
        self.udp.GetRecv(self.state)

    def send_command(self):
        """
        Send command to robot (only use in test mode & after setting command)
        """
        if time.time() - self.cmd_watchdog_timer > 0.2:
            self.cmd.mode = 0
            self.cmd.euler = [0, 0, 0]
            self.cmd.velocity = [0, 0]
            self.cmd.yawSpeed = 0.0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def getIMU(self):
        accel = self.state.imu.accelerometer
        gyro = self.state.imu.gyroscope
        quat = self.state.imu.quaternion
        rpy = self.state.imu.rpy
        return accel, gyro, quat, rpy

    def getJointStates(self):
        """Returns the joint angles (q) and velocities (dq) of the robot"""
        motorStates = self.state.motorState
        _q, _dq = zip(
            *[(motorState.q, motorState.dq) for motorState in motorStates[:12]]
        )
        q, dq = np.array(_q), np.array(_dq)

        return q, dq

    def getRemoteState(self):
        """Returns the state of the remote control"""
        wirelessRemote = self.state.wirelessRemote[:24]

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

    def setCommand(self, v_x, v_y, ω_z, bodyHeight=0.0, footRaiseHeight=0.0, mode=2):
        assert mode in [0, 2]  # Only mode 2: walking and mode 0: idea is allowed
        self.cmd_watchdog_timer = time.time()
        self.cmd.mode = mode
        self.cmd.bodyHeight = np.clip(bodyHeight, -0.15, 0.1)
        self.cmd.footRaiseHeight = np.clip(footRaiseHeight, -0.1, 0.1)
        _v_x, _v_y, _ω_z = self.clip_velocity(v_x, v_y, ω_z)
        self.cmd.velocity = [_v_x, _v_y]
        self.cmd.yawSpeed = _ω_z

    def close(self):
        self.running = False
        self.loop_thread.join()

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
