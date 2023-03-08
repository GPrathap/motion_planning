import cvxpy as cp
import numpy as np
import scipy as sp
from scipy import sparse

n = 10
x1 = 0.2
x2 = 0.6
sigma = .10
a = np.linspace(0, 1, n)
b = x1 + x2*a + sigma*np.random.standard_normal(a.shape)

theta = cp.Variable(2)

# X = np.row_stack((np.ones_like(b), a)).T 
# objective_function = (b - X*theta).T*(b-X*theta)
# obj = cp.Minimize(objective_function)
# prob = cp.Problem(obj)
# result = prob.solve()
# print(theta.value)


# objective = cp.Minimize(cp.sum_squares(theta[0]*a + theta[1] - b))
# prob = cp.Problem(objective)
# result = prob.solve()
# print(theta.value)
# print(x2, x1)



# Generate data
m = 20
n = 15
np.random.seed(1)
A = np.random.randn(m, n)
b = np.random.randn(m)

# Define and solve the CVXPY problem
x = cp.Variable(n)
cost = cp.sum_squares(A @ x - b)
prob = cp.Problem(cp.Minimize(cost))
prob.solve()

# Print result
print("\nThe optimal value is", prob.value)
print("The optimal x is")
print(x.value)
print("The norm of the residual is ", cp.norm(A @ x - b, p=2).value)


# Define problem
x = cp.Variable(n)
objective = cp.sum_squares(A@x - b)
prob = cp.Problem(cp.Minimize(objective))
# Using a different solver OSQP 
prob.solve(solver='OSQP', warm_start=True)

# Print result
print("\nThe optimal value is", prob.value)
print("The optimal x is")
print(x.value)
print("The norm of the residual is ", cp.norm(A @ x - b, p=2).value)
