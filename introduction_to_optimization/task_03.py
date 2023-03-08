import cvxpy as cp
import numpy as np
import scipy as sp
from scipy import sparse

# Generate problem data
sp.random.seed(1)
n = 15
m = 20
A = sparse.random(m, n, density=0.5)

x_true = np.multiply((np.random.rand(n) > 0.8).astype(float),
                     np.random.randn(n)) / np.sqrt(n)
b = A.dot(x_true) + 0.5*np.random.randn(m)
gammas = np.linspace(1, 10, 11)

# Define problem
x = cp.Variable(n)
gamma = cp.Parameter(nonneg=True)
objective = 0.5*cp.sum_squares(A@x - b) + gamma*cp.norm1(x)
prob = cp.Problem(cp.Minimize(objective))

# Solve problem for different values of gamma parameter
for gamma_val in gammas:
    gamma.value = gamma_val
    prob.solve()
    # prob.solve(solver='OSQP', warm_start=True)
    # Print result.
    print("\nThe optimal value is", prob.value)
    print("The optimal x is")
    print(x.value)
    print("The norm of the residual is ", cp.norm(A @ x - b, p=2).value)