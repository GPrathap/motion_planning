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
rho = 20
M = 0.1 # huber radius

# sole least-squares smoothing using huber loss
beta = cp.Variable(n)
obj = sum(cp.huber(beta-b, M)) + rho*cp.sum_squares(D@beta)
cp.Problem(cp.Minimize(obj)).solve()
beta = np.array(beta.value).flatten()

plt.plot(b,'o', alpha=.5)
plt.plot(beta,'r-', linewidth=2)

plt.show()