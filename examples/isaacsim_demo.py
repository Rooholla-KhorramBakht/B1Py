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

world = World()

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
        position=np.array([0, 0, 0.40]),
        physics_dt=1/400,
    )
)
world.reset()

for i in range(5000):
    # things run in sync
    print(b1.step(np.random.randn(12,)*10))
    world.step(render=True) # execute one physics step and one rendering step

simulation_app.close() # close Isaac Sim