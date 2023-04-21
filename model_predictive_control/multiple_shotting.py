"""
author: Geesara Prathap
email: ggeesara@gmail.com
license: BSD
"""
import numpy as np
from quadcopter import QuadCopter
from casadi import *
import casadi as ca

# Time horizon
T = 0.2
N = 20 # number of control intervals
robot_diam = 1.3
v_max = 0.2
v_min = -v_max
omega_max = ca.pi/4
omega_min = -omega_max
map_dim = [-100, 100, -100, 100, 0, 100]
xs = ca.DM([[5.8], [6], [4.0], [0.0]])

Q = ca.DM.zeros(4,4)
Q[0,0] = 1
Q[1,1] = 1
Q[2,2] = 1
Q[3,3] = 0.1
R = ca.DM.zeros(4,4)
R[0,0] = 0.5
R[1,1] = 0.5
R[2,2] = 0.5
R[3,3] = 0.05

obs_map = np.array([[-2.5, 1.5, 2, 2],[3.5, 5, 3, 2]])
obs_length = obs_map.shape[0]

# Declare model variables
x_ = ca.SX.sym('x')
y_ = ca.SX.sym('y')
z_ = ca.SX.sym('z')
theta = ca.SX.sym('theta')
x = ca.vertcat(x_, y_, z_, theta)
n_states = x.size()[0]

v_x = ca.SX.sym('v_x')
v_y = ca.SX.sym('v_y')
v_z = ca.SX.sym('v_z')
omega = ca.SX.sym('omega')

u = ca.vertcat(v_x, v_y, v_z, omega)
n_controls = u.size()[0]

# Model equations
xdot = ca.vertcat(v_x*ca.cos(theta)-v_y*ca.sin(theta), v_y*ca.cos(theta) + v_x*ca.sin(theta), v_z, omega)
x0_ = ca.DM([[0], [0], [0], [0]])
P = ca.SX.sym('P', n_states + n_states)
L = ca.mtimes((x-P[n_states:n_states*2]).T, ca.mtimes(Q,(x-P[n_states:n_states*2]))) + ca.mtimes(u.T, ca.mtimes(R, u)) 
f = ca.Function('f', [x, u, P], [xdot, L], ['x', 'u', 'p'], ['xdot', 'L'])

# Start with an empty NLP
w=[]
w0 = []
lbw = []
ubw = []
J = 1
g=[]
lbg = []
ubg = []

U = SX.sym('U', n_controls, N)
X = SX.sym('X', n_states, N+1)

g += [X[:,0] - P[0:n_states]]
lbg += [0, 0, 0, 0]
ubg += [0, 0, 0, 0]

ibj = 1
# Formulate the NLP
for k in range(N):
    # New NLP variable for the control
    Uk = U[:,k]
    Xk = X[:,k]
    # Integrate till the end of the interval
    x_dot_, l = f(Xk, Uk, P)
    Xk_end = x_dot_*T
    J=J+l
    st_next_euler = Xk + Xk_end
    # New NLP variable for state at end of interval
    Xk = X[:,k+1]
    # Add equality constraint
    g += [Xk-st_next_euler]
    lbg += [0, 0, 0, 0]
    ubg += [0, 0, 0, 0]
    ibj += 1

obs_count = 0
for k in range(0, N+1):
    st = X[:,k]
    for obs in obs_map:
        obs_cost = -sqrt((st[0]-obs[0])**2 + (st[1]-obs[1])**2 + (st[2]-obs[2])**2) + (obs[3] + robot_diam)
        g += [obs_cost]
        lbg += [-inf]
        ubg += [0]

OPT_variables = vertcat(reshape(X, n_states*(N+1), 1), reshape(U, n_controls*N, 1))
# Create an NLP solver

opts = {}
opts["expand"] = True
opts["ipopt.max_iter"] = 100
opts["ipopt.tol"] = 1e-4
opts["ipopt.print_level"] = 0
opts["print_time"] = 0
opts["ipopt.acceptable_tol"] = 1e-8

prob = {'f': J, 'x': OPT_variables, 'g': vertcat(*g), 'p':P}
solver = nlpsol('solver', 'ipopt', prob, opts)    

P_ = ca.vertcat(x0_, xs)

lbx = DM(n_states*(N+1)+n_controls*N,1)
ubx = DM(n_states*(N+1)+n_controls*N,1)

lbx[0:n_states*(N+1):n_states,0] = map_dim[0]
ubx[0:n_states*(N+1):n_states,0] = map_dim[1]
lbx[1:n_states*(N+1):n_states,0] = map_dim[2]
ubx[1:n_states*(N+1):n_states,0] = map_dim[3]
lbx[2:n_states*(N+1):n_states,0] = map_dim[4]
ubx[2:n_states*(N+1):n_states,0] = map_dim[5]
lbx[3:n_states*(N+1):n_states,0] = -inf
ubx[3:n_states*(N+1):n_states,0] = inf

lbx[n_states*(N+1):n_states*(N+1) + n_controls*N:n_controls,0] = v_min
ubx[n_states*(N+1):n_states*(N+1) + n_controls*N:n_controls,0] = v_max
lbx[n_states*(N+1)+1:n_states*(N+1) + n_controls*N:n_controls,0] = v_min
ubx[n_states*(N+1)+1:n_states*(N+1) + n_controls*N:n_controls,0] = v_max
lbx[n_states*(N+1)+2:n_states*(N+1) + n_controls*N:n_controls,0] = v_min
ubx[n_states*(N+1)+2:n_states*(N+1) + n_controls*N:n_controls,0] = v_max
lbx[n_states*(N+1)+1:n_states*(N+1) +n_controls*N:2,0] = omega_min
ubx[n_states*(N+1)+1:n_states*(N+1) +n_controls*N:2,0] = omega_max

t0 = 0
x0 = DM([[-5], [-5], [0], [0]])
xx = DM(n_states, 300)
xx[:,0] = x0
t = DM(1, 300)
t[0] = t0
u0 = DM.zeros(N, n_controls)
X0 = repmat(x0, 1, N+1).T



def shift(T, t0, x0, u, P, f):
    st = x0
    con = u[0,:].T 
    x_dot_, _ = f(st, con, P)
    st = st + (T*x_dot_)
    x0 = st
    t0 = t0 + T
    u_rest = u[1:u.size()[0]:1,:]
    u_last = u[u.size()[0]-1:u.size()[0]:1,:]
    u0 = vertcat(u_rest, u_last)
    return t0, x0, u0



args = {'lbx':lbx, 'ubx':ubx, 'lbg':lbg
    , 'ubg':ubg, 'p':[], 'x0':[0.5, 1.0]}


w0 =  vertcat(reshape(X0.T, n_states*(N+1), 1), reshape(u0.T, n_controls*N, 1))
sim_time = 40
mpciter = 0
prediction_horizon_poses = []
u_cl = []
current_state = []

while(norm_2(x0-xs)>1e-2 and mpciter < sim_time/T):
    args['p'] = vertcat(x0, xs)
    args['x0'] = vertcat(reshape(X0.T, n_states*(N+1), 1), reshape(u0.T, n_controls*N, 1))
    sol = solver(**args)
    u = reshape(sol['x'][n_states*(N+1):sol['x'].size()[0]:1].T, n_controls, N).T
    horizon_poses = reshape(sol['x'][0:n_states*(N+1):1].T, n_states, N+1).T.full()
    prediction_horizon_poses.append(horizon_poses)
    u_cl.append(u.full())
    t[mpciter+1] = t0
    t0, x0, u0 = shift(T, t0, x0, u, args['p'], f) 
    X0 = reshape(sol['x'][0:n_states*(N+1)].T, n_states, N+1).T
    xx[:,mpciter+2] = x0
    current_state.append(x0.full().flatten())
    X0 = reshape(sol['x'][0:n_states*(N+1):1].T, n_states, N+1).T
    x0_rest = X0[1:X0.size()[0]:1,:]
    x0_last = X0[X0.size()[0]-1:X0.size()[0]:1,:]
    X0 = vertcat(x0_rest, x0_last)
    mpciter = mpciter + 1

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]

df = np.array(current_state)
quadcopter = QuadCopter() 
def control_loop(i):
    state = np.array(df[i])
    return quadcopter.world_frame(np.append(state, 0), prediction_horizon_poses[i])
    
quadcopter.plot_quad_3d(control_loop, obs_map)

