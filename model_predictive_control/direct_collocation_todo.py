"""
author: Geesara Prathap
email: ggeesara@gmail.com
license: BSD
"""
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from quadcopter import QuadCopter
import numpy as np

# Degree of interpolating polynomial
d = 3

# Get collocation points
tau_root = np.append(0, ca.collocation_points(d, 'legendre'))

# Coefficients of the collocation equation
C = np.zeros((d+1,d+1))

# Coefficients of the continuity equation
D = np.zeros(d+1)

# Coefficients of the quadrature function
B = np.zeros(d+1)

# Construct polynomial basis
for j in range(d+1):
    # Construct Lagrange polynomials to get the polynomial basis at the collocation point
    p = np.poly1d([1])
    for r in range(d+1):
        if r != j:
            p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j]-tau_root[r])

    # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
    D[j] = p(1.0)

    # Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
    pder = np.polyder(p)
    for r in range(d+1):
        C[j,r] = pder(tau_root[r])

    # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
    pint = np.polyint(p)
    B[j] = pint(1.0)


# Time horizon
T = 0.2
N = 30 # number of control intervals
robot_diam = 1.3
v_max = 0.2
v_min = -v_max
omega_max = ca.pi/4
omega_min = -omega_max
map_dim = [-6, 6, -6, 6, 0, 6]
xs = ca.DM([[5.8], [5], [5.0], [0.0]])
x0 = ca.DM([[-5], [-5], [0], [0]])

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
P = ca.SX.sym('P', n_states + n_states)
f = ca.Function('f', [x, u], [xdot], ['x', 'u'], ['xdot'])

# Start with an empty NLP
w0=[]
J=0

lb_state = []
ub_state = []
lb_control = []
ub_control = []
# "Lift" initial conditions
U = ca.SX.sym('U', n_controls, N)
X = ca.SX.sym('X', n_states, N+1 + d*N)

Xk = X[:,0]

g = ca.SX.sym('g',n_states, (N+1+d*N))
g[:,0] = Xk - P[0:n_states]

# Formulate the NLP
pos_index = 0
constraints = 1

obs_count = 0
obs_g = ca.SX.sym('obs_g',(N+1+d*N)*obs_length, 1)
for obs in obs_map:
    obs_g[obs_count] = -ca.sqrt((Xk[0]-obs[0])**2 + (Xk[1]-obs[1])**2 + (Xk[2]-obs[2])**2) + (obs[3] + robot_diam)
    obs_count += 1

for k in range(N):
    # New NLP variable for the control
    Uk = U[:,k]

    # State at collocation points
    Xc = []
    for j in range(d):
        Xkj = X[:,pos_index+1+j]
        Xc.append(Xkj)

    pos_index += d+1
    # Loop over collocation points
    Xk_end = D[0]*Xk
    for j in range(1,d+1):
        # Expression for the state derivative at the collocation point
        xp = C[0,j]*Xk
        for r in range(d): xp = xp + C[r+1,j]*Xc[r]
        # Append collocation equations
        fj = f(Xc[j-1],Uk)
        qj = ca.mtimes((Xc[j-1]-P[n_states:n_states*2]).T, ca.mtimes(Q,(Xc[j-1]-P[n_states:n_states*2]))) + ca.mtimes(Uk.T, ca.mtimes(R, Uk)) 
        g[:,constraints] = T*fj - xp
        constraints += 1
        st = xp
        for obs in obs_map:
            obs_g[obs_count] = -ca.sqrt((st[0]-obs[0])**2 + (st[1]-obs[1])**2 + (st[2]-obs[2])**2) + (obs[3] + robot_diam)
            obs_count += 1
        # Add contribution to the end state
        Xk_end = Xk_end + D[j]*Xc[j-1]
        # Add contribution to quadrature function
        J = J + B[j]*qj*T

    Xk = X[:,pos_index]
    # Add equality constraint
    g[:,constraints] = Xk_end-Xk
    constraints += 1
    st = Xk
    for obs in obs_map:
        obs_g[obs_count] = -ca.sqrt((st[0]-obs[0])**2 + (st[1]-obs[1])**2 + (st[2]-obs[2])**2) + (obs[3] + robot_diam)
        obs_count += 1
    
# Concatenate vectors
g = ca.reshape(g, n_states*((N+1)+d*N), 1)
g = ca.vertcat(g, obs_g)

lbg = ca.DM(1, n_states*((N+1)+d*N) + ((N+1)+d*N)*obs_length)
ubg = ca.DM(1, n_states*((N+1)+d*N) + ((N+1)+d*N)*obs_length)

lbg[0,0:n_states*((N+1)+d*N):1] = 0
ubg[0,0:n_states*((N+1)+d*N):1] = 0

lbg[0,n_states*((N+1)+d*N):n_states*((N+1)+d*N) + ((N+1)+d*N)*obs_length:1] = -ca.inf
ubg[0,n_states*((N+1)+d*N):n_states*((N+1)+d*N) + ((N+1)+d*N)*obs_length:1] = 0


lbx = ca.DM(n_states*((N+1)+d*N)+n_controls*N,1)
ubx = ca.DM(n_states*((N+1)+d*N)+n_controls*N,1)

lbx[0:n_states*((N+1)+d*N):n_states,0] = map_dim[0]
ubx[0:n_states*((N+1)+d*N):n_states,0] = map_dim[1]
lbx[1:n_states*((N+1)+d*N):n_states,0] = map_dim[2]
ubx[1:n_states*((N+1)+d*N):n_states,0] = map_dim[3]
lbx[2:n_states*((N+1)+d*N):n_states,0] = map_dim[4]
ubx[2:n_states*((N+1)+d*N):n_states,0] = map_dim[5]
lbx[3:n_states*((N+1)+d*N):n_states,0] = -ca.inf
ubx[3:n_states*((N+1)+d*N):n_states,0] = ca.inf

lbx[n_states*((N+1)+d*N):n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_min
ubx[n_states*((N+1)+d*N):n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_max
lbx[n_states*((N+1)+d*N)+1:n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_min
ubx[n_states*((N+1)+d*N)+1:n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_max
lbx[n_states*((N+1)+d*N)+2:n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_min
ubx[n_states*((N+1)+d*N)+2:n_states*((N+1)+d*N) + n_controls*N:n_controls,0] = v_max
lbx[n_states*((N+1)+d*N)+3:n_states*((N+1)+d*N) +n_controls*N:n_controls,0] = omega_min
ubx[n_states*((N+1)+d*N)+3:n_states*((N+1)+d*N) +n_controls*N:n_controls,0] = omega_max

OPT_variables = ca.vertcat(ca.reshape(X, n_states*((N+1)+d*N), 1), ca.reshape(U, n_controls*N, 1))

# Create an NLP solver
prob = {'f': J, 'x': OPT_variables, 'g': g, 'p':P}

opts = {}
opts["expand"] = True
opts["ipopt.max_iter"] = 100
opts["ipopt.tol"] = 1e-4
opts["ipopt.print_level"] = 0
opts["print_time"] = 0
opts["ipopt.acceptable_tol"] = 1e-8

nlp_prob = {'f':J, 'x':OPT_variables, 'p':P, 'g':g}
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)


# # Solve the NLP
def shift(T, t0, x0, u, f):
    st = x0
    con = u[0,:].T 
    x_dot_= f(st, con)
    st = st + (T*x_dot_)
    x0 = st
    t0 = t0 + T
    u_rest = u[1:u.size()[0]:1,:]
    u_last = u[u.size()[0]-1:u.size()[0]:1,:]
    u0 = ca.vertcat(u_rest, u_last)
    return t0, x0, u0

args = {'lbx':lbx, 'ubx':ubx, 'lbg':lbg, 'ubg':ubg, 'p':[], 'x0':[0.5, 1.0]}

t0 = 0
sim_time = 80
mpciter = 0
prediction_horizon_poses = []
current_state = []
t = ca.DM(1, 500)
t[0] = t0

X0 = ca.repmat(x0, 1, (N+1+d*N))
u0 = ca.DM.zeros(N, n_controls)

while(ca.norm_2(x0-xs)>1e-2 and mpciter < sim_time/T):
    args['p'] = ca.vertcat(x0, xs)
    args['x0'] = ca.vertcat(ca.reshape(X0.T, n_states*((N+1)+N*d), 1), ca.reshape(u0.T, n_controls*N, 1))
   
    sol = solver(**args)
    u = ca.reshape(sol['x'][n_states*((N+1)+d*N):sol['x'].size()[0]:1].T, n_controls, N).T
    fgh = ca.reshape(sol['x'][0:n_states*((N+1)+N*d):1].T, n_states, (N+1)+N*d).T.full()
    prediction_horizon_poses.append(fgh)
    
    t[mpciter+1] = t0
    t0, x0, u0 = shift(T, t0, x0, u, f) 
    current_state.append(x0.full().flatten())
    X0 = ca.reshape(sol['x'][0:n_states*((N+1)+d*N):1].T, n_states, N+1 + d*N).T

    x0_rest = X0[1:X0.size()[0]:1,:]
    x0_last = X0[X0.size()[0]-1:X0.size()[0]:1,:]
    X0 = ca.vertcat(x0_rest, x0_last)
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







