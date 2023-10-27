import threading
import time

import numpy as np
import numpy.linalg as LA
import pinocchio as pin
import robot_interface as sdk

import hppfcl  # isort: skip


class B1HighLevelReal:
    def __init__(self, loop_frequency=100):
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

        package_directory = "/home/bolun/Documents/B1Py/B1Py/assets/urdf"
        urdf_path = package_directory + "/b1.urdf"
        self.pin_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_directory)

        # for calf collision detection
        self.calf_ids = {
            "FL": self.pin_robot.model.getFrameId("FL_calf"),
            "RL": self.pin_robot.model.getFrameId("RL_calf"),
            "FR": self.pin_robot.model.getFrameId("FR_calf"),
            "RR": self.pin_robot.model.getFrameId("RR_calf"),
        }

        # create leg cylinders hppfcl instances
        self.fl_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.rl_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.fr_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.rr_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.calf_offset = pin.SE3(np.eye(3), np.array([0.005237, 0.0, -0.15]))

        self.running = True
        self.loop_thread = threading.Thread(target=self.loop_fn)
        self.loop_thread.start()

        # for velocity clipping
        self.vx_max = 0.5
        self.vy_max = 0.4
        self.P_v_max = np.diag([1 / self.vx_max**2, 1 / self.vy_max**2])
        self.omega_max = 0.5
        self.omega_min = -0.5

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

    def getBatteryState(self):
        """Returns the battery percentage of the robot"""
        batteryState = self.state.bms

        return batteryState.SOC

    def setCommand(
        self, v_x, v_y, omega_z, bodyHeight=0.0, footRaiseHeight=0.0, mode=2
    ):
        assert mode in [0, 2], "Only mode 2: walking and mode 0: idea is allowed"
        self.cmd_watchdog_timer = time.time()
        self.cmd.mode = mode
        self.cmd.bodyHeight = np.clip(bodyHeight, -0.15, 0.1)
        self.cmd.footRaiseHeight = np.clip(footRaiseHeight, -0.1, 0.1)
        _v_x, _v_y = self.clip_velocity(v_x, v_y)
        self.cmd.velocity = [_v_x, _v_y]
        self.cmd.yawSpeed = omega_z

    def close(self):
        self.running = False
        self.loop_thread.join()

    def check_calf_collision(self, q):
        q_pin = self.q_map_b12pin(q)

        self.pin_robot.framesForwardKinematics(q_pin)

        fl_calf_pose = self.pin_robot.data.oMf[self.calf_ids["FL"]] * self.calf_offset
        rl_calf_pose = self.pin_robot.data.oMf[self.calf_ids["RL"]] * self.calf_offset
        fr_calf_pose = self.pin_robot.data.oMf[self.calf_ids["FR"]] * self.calf_offset
        rr_calf_pose = self.pin_robot.data.oMf[self.calf_ids["RR"]] * self.calf_offset

        # compute distance between calf cylinders
        req = hppfcl.CollisionRequest()
        res = hppfcl.CollisionResult()

        T_fl = hppfcl.Transform3f(fl_calf_pose.rotation, fl_calf_pose.translation)
        T_rl = hppfcl.Transform3f(rl_calf_pose.rotation, rl_calf_pose.translation)
        T_fr = hppfcl.Transform3f(fr_calf_pose.rotation, fr_calf_pose.translation)
        T_rr = hppfcl.Transform3f(rr_calf_pose.rotation, rr_calf_pose.translation)

        col_fl_rl = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.rl_cylinder_hppfcl, T_rl, req, res
        )
        col_fl_fr = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.fr_cylinder_hppfcl, T_fr, req, res
        )
        col_fl_rr = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        col_rl_fr = hppfcl.collide(
            self.rl_cylinder_hppfcl, T_rl, self.fr_cylinder_hppfcl, T_fr, req, res
        )
        col_rl_rr = hppfcl.collide(
            self.rl_cylinder_hppfcl, T_rl, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        col_fr_rr = hppfcl.collide(
            self.fr_cylinder_hppfcl, T_fr, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        in_collision = np.all(
            [col_fl_rl, col_fl_fr, col_fl_rr, col_rl_fr, col_rl_rr, col_fr_rr]
        )

        return in_collision

    def q_map_b12pin(self, q):
        """
        B1 gives the joint angles in the order of
        FR, FL, RR, RL (hip, thigh, calf)

        Pinocchio expects the joint angles in the order of
        FL, FR, RL, RR (hip, thigh, calf)

        This function maps the joint angles from B1 to Pinocchio
        """
        q_fr = q[:3]
        q_fl = q[3:6]
        q_rr = q[6:9]
        q_rl = q[9:]

        q_pin = np.concatenate([q_fl, q_fr, q_rl, q_rr])

        return q_pin

    def clip_velocity(self, v_x, v_y):
        _v = np.array([[v_x], [v_y]])
        _scale = np.sqrt(_v.T @ self.P_v_max @ _v)
        scale = np.clip(_scale, 0, 1)

        return scale * v_x, scale * v_y
