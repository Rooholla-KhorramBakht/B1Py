from omni.isaac.core.utils.stage import get_current_stage, get_stage_units
import numpy as np
from typing import Optional, List
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
from omni.isaac.sensor import ContactSensor, IMUSensor
from collections import deque
import numpy as np
from omni.isaac.core.articulations import Articulation
from dataclasses import field, dataclass
import numpy as np
from omni.isaac.quadruped.utils.types import NamedTuple, FrameState
import B1Py

@dataclass
class B1State(NamedTuple):
    """The kinematic state of the articulated robot."""

    base_frame: FrameState = field(default_factory=lambda: FrameState("root"))
    """State of base frame"""

    joint_pos: np.ndarray = field(default_factory=lambda: np.zeros(12))
    """Joint positions with shape: (12,)"""

    joint_vel: np.ndarray = field(default_factory=lambda: np.zeros(12))
    """Joint positions with shape: (12,)"""


@dataclass
class B1Measurement(NamedTuple):
    """The state of the robot along with the mounted sensor data."""

    state: B1State = field(default=B1State)
    """The state of the robot."""

    foot_forces: np.ndarray = field(default_factory=lambda: np.zeros(4))
    """Feet contact force of the robot in the order: FL, FR, RL, RR."""

    base_lin_acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    """Accelerometer reading from IMU attached to robot's base."""

    base_ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    """Gyroscope reading from IMU attached to robot's base."""


@dataclass
class B1Command(NamedTuple):
    """The command on the robot actuators."""

    desired_joint_torque: np.ndarray = field(default_factory=lambda: np.zeros(12))
    """Desired joint positions of the robot: (12,)"""


class B1SimLowLevel(Articulation):

    def __init__(
        self,
        prim_path: str,
        name: str = "unitree_quadruped",
        physics_dt: Optional[float] = 1 / 400.0,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        [Summary]
        
        initialize robot, set up sensors and interfaces
        
        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            physics_dt {float} -- physics downtime of the controller
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot        
        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            prim.GetReferences().AddReference(B1Py.USD_PATH)

        self._measurement = B1Measurement()
        self._command = B1Command()
        self._state = B1State()
        self._default_b1_state = B1State()

        if position is not None:
            self._default_b1_state.base_frame.pos = np.asarray(position)
        else:
            self._default_b1_state.base_frame.pos = np.array([0.0, 0.0, 0.0])

        self._default_b1_state.base_frame.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._default_b1_state.base_frame.ang_vel = np.array([0.0, 0.0, 0.0])
        self._default_b1_state.base_frame.lin_vel = np.array([0.0, 0.0, 0.0])
        self._default_b1_state.joint_pos = np.array([0.56, 1.05, -2.64, 0.56, 1.05, -2.64, -0.56, 1.05, -2.64, -0.56, 1.05, -2.64])
        self._default_b1_state.joint_vel = np.zeros(12)
        
        self._goal = np.zeros(3)
        self.meters_per_unit = get_stage_units()

        super().__init__(prim_path=self._prim_path, name=name, position=position, orientation=orientation)

        # contact sensor setup
        self.feet_order = ["FL", "RL", "FR", "RR"]
        self.feet_path = [
            self._prim_path + "/FL_foot",
            self._prim_path + "/FR_foot",
            self._prim_path + "/RL_foot",
            self._prim_path + "/RR_foot",
        ]

        self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]
        self._contact_sensors = [None] * 4
        for i in range(4):
            self._contact_sensors[i] = ContactSensor(
                prim_path=self.feet_path[i] + "/sensor",
                min_threshold=0,
                max_threshold=1000000,
                radius=0.03,
                dt=physics_dt,
            )

        self.foot_force = np.zeros(4)
        self.enable_foot_filter = True
        self._FILTER_WINDOW_SIZE = 20
        self._foot_filters = [deque(), deque(), deque(), deque()]

        # Translation maps between Isaac and Bullet
        self.bullet_joint_order = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                                   'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                                   'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                                   'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        self.isaac_joint_order = [
        'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'
        ]
        
        self.isaac_name_2_index = {s:i for i,s in enumerate(self.isaac_joint_order)}
        self.bullet_name_2_index = {s:i for i,s in enumerate(self.bullet_joint_order)}
        
        self.to_bullet_index = np.array([self.isaac_name_2_index[id] for id in self.bullet_joint_order])
        self.to_isaac_index =  np.array([self.bullet_name_2_index[id] for id in self.isaac_joint_order])
        # imu sensor setup
        self.imu_path = self._prim_path + "/imu_link"
        self._imu_sensor = IMUSensor(
            prim_path=self.imu_path + "/imu_sensor",
            name="imu",
            dt=physics_dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )
        self.base_lin = np.zeros(3)
        self.ang_vel = np.zeros(3)
        return

    def toIsaacOrder(self, x):
        return x[self.to_isaac_index,...]
    
    def toBulletOrder(self,x):
        return x[self.to_bullet_index,...]
    
    def set_state(self, state: B1State) -> None:
        """[Summary]
        
        Set the kinematic state of the robot.

        Args:
            state {B1State} -- The state of the robot to set.

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        self.set_world_pose(position=state.base_frame.pos, orientation=state.base_frame.quat[[3, 0, 1, 2]])
        self.set_linear_velocity(state.base_frame.lin_vel)
        self.set_angular_velocity(state.base_frame.ang_vel)
        
        self.set_joint_positions(
            positions=np.asarray(self.toIsaacOrder(state.joint_pos), dtype=np.float32)
        )
        self.set_joint_velocities(
            velocities=np.asarray(self.toIsaacOrder(state.joint_vel), dtype=np.float32)
        )
        self.set_joint_efforts(np.zeros_like(state.joint_pos))
        return

    def update_contact_sensor_data(self) -> None:
        """[summary]
        
        Updates processed contact sensor data from the robot feets, store them in member variable foot_force
        """
        # Order: FL, RL, FR, RR
        for i in range(len(self.feet_path)):
            frame = self._contact_sensors[i].get_current_frame()
            if "force" in frame:
                if self.enable_foot_filter:
                    self._foot_filters[i].append(frame["force"])
                    if len(self._foot_filters[i]) > self._FILTER_WINDOW_SIZE:
                        self._foot_filters[i].popleft()
                    self.foot_force[i] = np.mean(self._foot_filters[i])

                else:
                    self.foot_force[i] = frame["force"]

    def update_imu_sensor_data(self) -> None:
        """[summary]
        
        Updates processed imu sensor data from the robot body, store them in member variable base_lin and ang_vel
        """
        frame = self._imu_sensor.get_current_frame()
        self.base_lin = frame["lin_acc"]
        self.ang_vel = frame["ang_vel"]
        return

    def update(self) -> None:
        """[summary]
        
        update robot sensor variables, state variables in B1Measurement
        """

        self.update_contact_sensor_data()
        self.update_imu_sensor_data()

        # joint pos and vel from the DC interface
        self.joint_state = super().get_joints_state()

        self._state.joint_pos = self.toBulletOrder(self.joint_state.positions)
        self._state.joint_vel = self.toBulletOrder(self.joint_state.velocities)

        # base frame
        base_pose = self.get_world_pose()
        self._state.base_frame.pos = base_pose[0]
        self._state.base_frame.quat = base_pose[1][[1, 2, 3, 0]]
        self._state.base_frame.lin_vel = self.get_linear_velocity()
        self._state.base_frame.ang_vel = self.get_angular_velocity()

        # assign to _measurement obj
        self._measurement.state = self._state
        self._measurement.foot_forces = np.asarray(self.foot_force)
        self._measurement.base_ang_vel = np.asarray(self.ang_vel)
        self._measurement.base_lin_acc = np.asarray(self.base_lin)
        return
    
    def read_states(self):
        """[summary]
        reads the state of the robot
        Returns:
        B1Measurement -- The state of the robot.
        """
        self.update()
        return self._measurement
    
    def set_action(self, action):
        """[summary]
        sets the joint torques
        Argument:
            action {np.ndarray} -- Joint torque command
        """
        self.set_joint_efforts(np.asarray(self.toIsaacOrder(action), dtype=np.float32))
        return 
    
    def step(self,action):
        """[summary]
        compute desired torque and set articulation effort to robot joints
        
        Argument:
            action {np.ndarray} -- Joint torque command
        Returns:
        np.ndarray -- The desired joint torques for the robot.
        """
        self.update()
        self.set_joint_efforts(np.asarray(self.toIsaacOrder(action), dtype=np.float32))
        return self._measurement

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize dc interface, set up drive mode and initial robot state
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("effort")
        self.set_state(self._default_b1_state)
        for i in range(4):
            self._contact_sensors[i].initialize()
        return

    def post_reset(self) -> None:
        """[summary]

        post reset articulation and qp_controller
        """
        super().post_reset()
        for i in range(4):
            self._contact_sensors[i].post_reset()
        self.set_state(self._default_b1_state)
        return

