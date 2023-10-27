# launch Isaac Sim before any other imports
# default first two lines in any standalone application
import sys

import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path

from B1Py.sim.isaac.b1 import UnitreeB1

sys.path.append("/home/vr-station/projects/lcm/build/python")

import time

from B1Py.lcm_bridges import LCMBridgeServer
from B1Py.lcm_types.unitree_lowlevel import UnitreeLowCommand
from B1Py.sim.utils import simulationManager
from B1Py.utils import NumpyMemMapDataPipe

PHYSICS_DT = 1 / 100
RENDERING_DT = 1 / 100

world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = get_prim_at_path("/World/Warehouse")
if not prim.IsValid():
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

# spawn a B1 robot
b1 = world.scene.add(
    UnitreeB1(
        prim_path="/World/B1",
        name="B1",
        position=np.array([0, 0, 0.8]),
        physics_dt=PHYSICS_DT,
    )
)

# lidar = world.scene.get_object("/World/B1/imu_link/Lidar")
# breakpoint()
world.reset()
# b1.disable_gravity()
b1.initialize()
# lidar.add_point_cloud_data_to_frame()
lcm_server = LCMBridgeServer(robot_name="b1")

cmd_stamp = time.time()
cmd_stamp_old = cmd_stamp

cmd = UnitreeLowCommand()
cmd.kd = 12 * [2.5]
cmd.kp = 12 * [100]
cmd.dq_des = 12 * [0]
cmd.q_des = b1.init_joint_pos

sim_manager = simulationManager(
    robot=b1,
    lcm_server=lcm_server,
    default_cmd=cmd,
    physics_dt=PHYSICS_DT,
    lcm_timeout=1e-4,
)
lidar_data_pipe = NumpyMemMapDataPipe(
    "lidar_data_pipe", force=True, dtype="float32", shape=(300, 16, 3)
)
counter = 0
while simulation_app.is_running():
    # sim_manager.step(counter*PHYSICS_DT)
    # Step the world with rendering 50 times per second
    sim_manager.step(counter * PHYSICS_DT)
    if counter % 2 == 0:
        world.step(render=True)
        lidar_data_pipe.write(np.random.rand(300, 16, 3))
        # print(lidar.get_current_frames().keys())

    else:
        world.step(render=False)

    counter += 1
simulation_app.close()  # close Isaac Sim