"""
author: Geesara Prathap
email: ggeesara@gmail.com
license: BSD
"""
from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse

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

umin = np.array([-0.1, -0.1, -0.1])
umax = np.array([0.1, 0.1, 0.1])
xmin = np.array([-6,-6,-6])
xmax = np.array([ 6, 6, 6])

Q = sparse.diags([1., 1., 1.])
QN = Q
R = 1.0*sparse.eye(3)

x0 = np.array([5,5, 1.])
xr = np.array([-5,-5.,5.])

N = 20

u = Variable((nu, N))
x = Variable((nx, N+1))
x_init = Parameter(nx)
objective = 0
constraints = [x[:,0] == x_init]
for k in range(N):
    objective += quad_form(x[:,k] - xr, Q) + quad_form(u[:,k], R)
    constraints += [x[:,k+1] == Ad@x[:,k] + Bd@u[:,k]]
    constraints += [xmin <= x[:,k], x[:,k] <= xmax]
    constraints += [umin <= u[:,k], u[:,k] <= umax]

objective += quad_form(x[:,N] - xr, QN)
prob = Problem(Minimize(objective), constraints)


current_state = []
prediction_horizon_poses = []
nsim = 200

for i in range(nsim):
    x_init.value = x0
    prob.solve(solver=OSQP, warm_start=True)
    x0 = Ad.dot(x0) + Bd.dot(u[:,0].value)