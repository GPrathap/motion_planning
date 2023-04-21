"""
author: Geesara Prathap
email: ggeesara@gmail.com
license: BSD
"""

import numpy as np
from quadcopter import QuadCopter

obs_map = np.array([[-2.5, 1.5, 2, 2],[3.5, 5, 3, 2]])
obs_length = obs_map.shape[0]
source_dir = "/root/catkin_ws/src/motion_planning/motion_planning_alg/data/"

prediction_horizon_poses = np.load(source_dir + "prediction_horizon_poses.npy")
current_state = np.load(source_dir + 'current_state.npy')
points = prediction_horizon_poses.shape[0]
num_states = prediction_horizon_poses.shape[1]

current_state = np.reshape(current_state, (points, -1))
prediction_horizon_poses = np.reshape(prediction_horizon_poses, (points, -1, num_states))

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]

df = current_state

quadcopter = QuadCopter() 
def control_loop(i):
    state = np.array(df[i])
    return quadcopter.world_frame(np.append(state, 0), prediction_horizon_poses[i])
    
quadcopter.plot_quad_3d(control_loop, obs_map)

