import threading
import numpy as np
import robot_interface as sdk
import time
import numpy.linalg as LA
import time

class B1HighLevelReal():
    def __init__(self, loop_frequency= 100):
        self.loop_frequency = loop_frequency
        HIGHLEVEL = 0xee
        self.udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.220", 8082)
        self.cmd = sdk.HighCmd()
        self.state = sdk.HighState()
        self.udp.InitCmdData(self.cmd)
        self.cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
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
        self.running = True
        self.loop_thread = threading.Thread(target = self.loop_fn)
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
            time.sleep(1/self.loop_frequency)
            # Communication link is not ready at the begining of transaction
            # To know it is ready make sure the IMU data makes sense
            if not self.ready and LA.norm(self.getIMU()[0]) > 5:
                self.ready = True

    def getIMU(self):
        accel = self.state.imu.accelerometer
        gyro  = self.state.imu.gyroscope
        q = self.state.imu.quaternion
        rpy = self.state.imu.rpy
        return accel, gyro, q, rpy

    def getJointStates(self):
        return None

    def getBatteryState(self):
        return None

    def setCommand(self, v_x, v_y, omega_z,  bodyHeight = 0.0, footRaiseHeight = 0.0, mode = 2):
        assert mode in [0, 2], 'Only mode2: walking and mode0: idea is allowed' 
        self.cmd_watchdog_timer = time.time()
        self.cmd.mode = mode
        self.cmd.bodyHeight = np.clip(bodyHeight, -0.15, 0.1)
        self.cmd.footRaiseHeight = np.clip(footRaiseHeight, -0.1, 0.1)
        self.cmd.velocity = [v_x, v_y]
        self.cmd.yawSpeed = omega_z

    def close(self):
        self.running = False
        self.loop_thread.join()