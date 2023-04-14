"""
author: Geesara Prathap
email: ggeesara@gmail.com
license: BSD
"""

import osqp
import numpy as np
import scipy as sp
from scipy import sparse
from quadcopter import QuadCopter

# Discrete time model of a quadcopter
Ad = sparse.csc_matrix([
  [1.,      0.,     0.],
  [0.,      1.,     0.],
  [0.,      0.,     1.]
])

Bd = sparse.csc_matrix([
  [1.,  0,  0.],
  [0,  1.,  0],
  [0,  0, 1]])

[nx, nu] = Bd.shape

# Constraint
umin = np.array([-0.1, -0.1, -0.1])
umax = np.array([0.1, 0.1, 0.1])
xmin = np.array([-6,-6,-6])
xmax = np.array([ 6, 6, 6])

# Objective function
Q = sparse.diags([1., 1., 1.])
QN = Q
R = 1.0*sparse.eye(3)

# Initial and reference states
x0 = np.array([5,5, 1.])
xr = np.array([-5,-2.,2.])

# Prediction horizon
N = 20

# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)], format='csc')

# - linear objective
q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
               np.zeros(N*nu)])

# - linear dynamics
Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)

# - input and state constraints
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(N*nx)])
ueq = leq
Aineq = sparse.eye((N+1)*nx + N*nu)
lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

# - OSQP constraints
A = sparse.vstack([Aeq, Aineq], format='csc')
l = np.hstack([leq, lineq])
u = np.hstack([ueq, uineq])

# Create an OSQP object
prob = osqp.OSQP()

# # Setup workspace
prob.setup(P, q, A, l, u, warm_start=True)

# Simulate in closed loop
current_state = []
prediction_horizon_poses = []
nsim = 200
for i in range(nsim):
    # Solve
    res = prob.solve()
    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')

    # Apply first control input to the plant
    ctrl = res.x[-N*nu:-(N-1)*nu]
    x0_i = x0 
    next_pose = x0[0:3]
    x0 = Ad.dot(x0) + Bd.dot(ctrl)

    ctrl_commands = res.x[(N+1)*(nx):]
    ctrl_index = 0
    horizon_poses = []
    horizon_poses.append(np.append(next_pose, 0))
    for i in range(0, N):
        ctrl_i = ctrl_commands[ctrl_index:ctrl_index+3]

        next_pose = Ad.dot(next_pose) + Bd.dot(ctrl_i)
        horizon_poses.append(np.append(next_pose, 0))
        ctrl_index += 3
    prediction_horizon_poses.append(horizon_poses)    

    l[:nx] = -x0
    u[:nx] = -x0

    prob.update(l=l, u=u)
    current_state.append(x0[0:3])



prediction_horizon_poses = np.array(prediction_horizon_poses)

animation_frequency = 1
control_frequency = 20
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]
df = np.array(current_state)
quadcopter = QuadCopter() 
def control_loop(i):
    state = np.array(df[i])
    return quadcopter.world_frame(np.append(state, 0), prediction_horizon_poses[i])
    
obs_map = np.array([[-2.5, 1.5, 2, 2]])
quadcopter.plot_quad_3d(control_loop, obs_map)