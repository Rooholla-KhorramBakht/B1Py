from go1_gym_deploy.rl.walk_these_ways import *

if __name__ == '__main__':
    checkpoint_path = "../checkpoints/B1"
    
    cfg = loadParameters(checkpoint_path)
    policy = Policy(checkpoint_path)
    command_profile = CommandInterface()

    isaacsim_agent = IsaacSimAgent(cfg, command_profile)
    isaacsim_agent = HistoryWrapper(isaacsim_agent)

    obs = isaacsim_agent.reset()
    for i in range(1000000):
        policy_info = {}
        action = policy(obs, policy_info)
        if i%2==0:
            obs, ret, done, info = isaacsim_agent.step(action)
        # else:
            # isaacsim_agent.get_obs()