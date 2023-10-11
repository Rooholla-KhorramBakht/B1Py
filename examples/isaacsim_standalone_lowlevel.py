#launch Isaac Sim before any other imports
#default first two lines in any standalone application

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
import numpy as np
from B1Py.simulators.isaacsim import B1SimLowLevel
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path

PHYSICS_DT = 1/1000
RENDERING_DT = 1/60

world = World(physics_dt = PHYSICS_DT, rendering_dt = RENDERING_DT)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = get_prim_at_path("/World/Warehouse")
if not prim.IsValid():
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

# spawn a B1 robot
b1 = world.scene.add(
    B1SimLowLevel(
        prim_path="/World/B1",
        name="B1",
        position=np.array([0, 0, 0.2]),
        physics_dt=PHYSICS_DT,
    )
)
world.reset()
# b1.disable_gravity()
b1.initialize()


standup_phase1_q = np.array([0.0, 1.45, -2.6] * 4)
standup_phase2_q = np.array([0.0, 1, -1.5] * 4)

for i in range(5000):
    # things run in sync
    state = b1.read_states()
    q = state.state.joint_pos
    dq = state.state.joint_vel
    if i < 1000:
        q_des = standup_phase1_q
    else:
        q_des = standup_phase2_q
    action = (q_des-q)*140+ (0-dq)*5
    b1.set_action(action)
    world.step(render=True) 
    # execute one physics step and one rendering step

simulation_app.close() # close Isaac Sim