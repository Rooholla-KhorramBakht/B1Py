# launch Isaac Sim before any other imports
# default first two lines in any standalone application
import sys
import numpy as np
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})  # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.sensor import RotatingLidarPhysX
from B1Py.sim.isaac.b1 import UnitreeB1
from B1Py.sim.isaac.utils import AnnotatorManager
import cv2

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

# Add Lidar
lidar = world.scene.add(
    RotatingLidarPhysX(prim_path="/World/B1/imu_link/lidar", 
                       name="lidar", 
                       translation=[0.16,0.,0.14],
                       orientation=[0., 0., 0., 1.])
)
lidar.add_depth_data_to_frame()
lidar.add_point_cloud_data_to_frame()
lidar.set_rotation_frequency(0)
lidar.set_resolution([0.4,2])
# lidar.enable_visualization()
lidar.prim.GetAttribute("highLod").Set(True)
lidar.prim.GetAttribute("highLod").Set(True)

world.reset()
b1.initialize()

# Add cameras
# Create cameras
ann = AnnotatorManager(world)
ann.registerCamera("/World/B1/imu_link", "camera_left", (0.485, 0.0258, 0.028), (-90., 180., 90.), resolution=(640, 480))   
ann.registerCamera("/World/B1/imu_link", "camera_right", (0.485, 0.0258-0.05, 0.028), (-90., 180., 90.), resolution=(640, 480))   
ann.registerAnnotator('rgb', 'camera_left')
ann.registerAnnotator('rgb', 'camera_right')
ann.registerAnnotator('distance_to_camera', 'camera_left')
ann.setFocalLength('camera_left', 28)
ann.setClippingRange('camera_left', 0.2, 1000000.0)
ann.setFocalLength('camera_right', 28)
ann.setClippingRange('camera_right', 0.2, 1000000.0)

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
    "lidar_data_pipe", force=True, dtype="float32", shape=(900, 16, 3)
)
left_rgb_data_pipe = NumpyMemMapDataPipe(
    "left_rgb_data_pipe", force=True, dtype="uint8", shape=(480, 640, 4)
)
right_rgb_data_pipe = NumpyMemMapDataPipe(
    "right_rgb_data_pipe", force=True, dtype="uint8", shape=(480, 640, 4)
)
depth_data_pipe = NumpyMemMapDataPipe(
    "depth_data_pipe", force=True, dtype="float32", shape=(480, 640)
)
counter = 0
while simulation_app.is_running():
    # sim_manager.step(counter*PHYSICS_DT)
    # Step the world with rendering 50 times per second
    sim_manager.step(counter * PHYSICS_DT)
    if counter % 2 == 0:
        world.step(render=True)
        pc = lidar.get_current_frame()['point_cloud']
        lidar_data_pipe.write(pc, match_length=True)

        img_left = ann.getData('camera_left:rgb')
        img_right = ann.getData('camera_right:rgb')
        depth_left = ann.getData('camera_left:distance_to_camera')
        if img_left.shape[0]!=0 and img_right.shape[0]!=0:
            # cv2.imshow(f"rgb-left", img_left)
            # cv2.imshow(f"rgb-right", img_right)
            # cv2.imshow(f"depth", depth_left)
            left_rgb_data_pipe.write(img_left, match_length=False)
            right_rgb_data_pipe.write(img_right, match_length=False)
            depth_data_pipe.write(depth_left, match_length=False)
        # cv2.waitKey(1)
    else:
        world.step(render=False)

    counter += 1
simulation_app.close()  # close Isaac Sim
