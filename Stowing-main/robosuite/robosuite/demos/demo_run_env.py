import numpy as np
import robosuite as suite
from robosuite import load_controller_config

# controllers = {
#     "OSC_POSITION": [4, 3, 0.1],
# }

# Define controller path to load
controller_config = load_controller_config(default_controller="OSC_POSITION")

# create environment instance
env = suite.make(
    env_name="Stow", # try with other tasks like "Stack" and "Door"
    robots="Kinova3",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    controller_configs=controller_config,
    render_camera="agentview",
)

# reset the environment
env.reset()

for i in range(1000):
    action = np.zeros(4) # sample random action
    obs, reward, done, info = env.step(action)  # take action in the environment
    env.render()  # render on display

env.close()
