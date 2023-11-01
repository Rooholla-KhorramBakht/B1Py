import pypose as pp

from B1Py.controllers.RL.walk_these_ways.walk_these_ways import (
    CommandInterface,
    HistoryWrapper,
    IsaacSimAgent,
    Policy,
    loadParameters,
)

if __name__ == "__main__":
    checkpoint_path = "checkpoints/B1"

    cfg = loadParameters(checkpoint_path)
    policy = Policy(checkpoint_path)
    command_profile = CommandInterface()

    isaacsim_agent = IsaacSimAgent(cfg, command_profile)
    isaacsim_agent = HistoryWrapper(isaacsim_agent)
    control_dt = cfg["control"]["decimation"] * cfg["sim"]["dt"]
    simulation_dt = 1 / 100
    obs = isaacsim_agent.reset()
    for i in range(1000000):
        policy_info = {}
        action = policy(obs, policy_info)
        if i % (control_dt // simulation_dt) == 0:
            obs, ret, done, info = isaacsim_agent.step(action)
        else:
            isaacsim_agent.wait_for_state()
        yaw = pp.SO3(isaacsim_agent.state.quaternion).euler()[-1].item()
        # command_profile.yaw_vel_cmd = 3.1415/2-yaw * 1.5
        # command_profile.x_vel_cmd = 0*-i * 0.001 - 0.4 * isaacsim_agent.state.gt_pos[0]
        # command_profile.y_vel_cmd = -1.0 * isaacsim_agent.state.gt_pos[1]
        command_profile.yaw_vel_cmd = 0.2
        command_profile.x_vel_cmd = 0.3
        command_profile.y_vel_cmd = 0
