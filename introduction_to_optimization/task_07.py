import cvxpy as cp
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pylab as plt

beta = cp.Variable(2)
objective  = cp.Minimize(0)
prob = cp.Problem(objective=objective, constraints=[4*beta[0]+0.5*beta[1] == 23, -4*beta[0]+8*beta[1] == 34])
prob.solve()
beta = np.array(beta.value)

print(beta)
print(4*beta[0]+0.5*beta[1], " ", 23)
print(-4*beta[0]+8*beta[1], " ", 34)
