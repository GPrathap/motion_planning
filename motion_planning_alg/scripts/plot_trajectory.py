
import numpy as np 
import matplotlib.pyplot as plt

ref_x = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/reference_x.npy")
ref_y = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/reference_y.npy")
waypoints_x = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/waypoints_x.npy")
waypoints_y = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/waypoints_y.npy")

ref_k = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/reference_curvature.npy")
ref_yaw = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/reference_yaw.npy")

ref_x = np.resize(ref_x, (-1))
ref_y = np.resize(ref_y, (-1))

waypoints_x = np.resize(waypoints_x, (-1))
waypoints_y = np.resize(waypoints_y, (-1))

ref_k = np.resize(ref_k, (-1))
ref_yaw = np.resize(ref_yaw, (-1))

traversed_path_x = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/traversed_path_x.npy")
traversed_path_y = np.load("/root/catkin_ws/src/motion_planning/motion_planning_alg/data/traversed_path_y.npy")

fig, ax = plt.subplots(3, 1, figsize=(40,40))

ax[0].scatter(ref_x, ref_y, label='reference path')
ax[0].scatter(waypoints_x, waypoints_y, s=100, label="waypoints")
ax[0].legend()

ax[1].plot(ref_k, label="ref curvature")
ax[1].legend()
ax[2].plot(ref_yaw, label="ref yaw")
ax[2].legend()

fig, ax = plt.subplots(1, 1, figsize=(40,40))
ax.scatter(ref_x, ref_y, label='reference path')
ax.scatter(traversed_path_x, traversed_path_y, label='traversed path')
plt.show()