from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.franka.tasks import PickPlace
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core import SimulationContext
import numpy as np

my_world = World(stage_units_in_meters=1.0)
my_task = PickPlace()
my_world.add_task(my_task)
simulation_context = SimulationContext()

my_world.reset()
task_params = my_task.get_params()
my_franka = my_world.scene.get_object(task_params["robot_name"]["value"])
my_franka.disable_gravity()

articulation_controller = my_franka.get_articulation_controller()
articulation_controller.set_effort_modes("force")
articulation_controller.switch_control_mode("effort")

q_des = np.array(
    [
        np.pi/4,
        -0.785398163,
        0.0,
        -2.35619449,
        0.0,
        1.57079632679,
        0.785398163397,
    ]
)

i = 0
while simulation_app.is_running():
    my_world.step(render=True)

    q = my_franka.get_joint_positions()
    dq = my_franka.get_joint_velocities()

    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()

        t = simulation_context.current_time

        # breakpoint()
        tau = 0.01 * (q_des - q[:7]) - 0.03 * dq[:7]
        tau = np.concatenate([tau, np.zeros(2)])

        actions = ArticulationAction(
            joint_positions=None,
            joint_velocities=None,
            joint_efforts=tau,
        )

        articulation_controller.apply_action(actions)
simulation_app.close()