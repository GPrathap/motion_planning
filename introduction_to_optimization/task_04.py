import cvxpy as cp
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pylab as plt
n = 100
a = np.linspace(0, 3*np.pi, n)
b = np.sin(a)
b = b + np.random.randn(n)*.2

def diff(n, k=1):
    D = np.diag(-1*np.ones(n)) + np.diag(np.ones(n-1),1)
    D = D[:-1,:]
    if k > 1:
        return diff(n-1,k-1).dot(D)
    else:
        return D


D = diff(n, 2)

rho = 1
# construct and solve problem in cvxpy
beta = cp.Variable(n)
cp.Problem(cp.Minimize(cp.sum_squares(beta-b)+ rho*cp.sum_squares(D@beta))).solve()
beta = np.array(beta.value).flatten()

plt.figure()
plt.plot(b,'o',label='$b_i$')
plt.plot(beta,'g+-',label='$beta_i$')
plt.xlabel('$a$')
plt.ylabel('$b$')
plt.legend()
plt.show()

