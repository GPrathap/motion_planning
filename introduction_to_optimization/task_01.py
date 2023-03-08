import cvxpy as cp
import numpy as np
from scipy.optimize import linprog


x = cp.Variable(shape=(2,1), name="x")
A = np.array([[9,3],[-7,9]])

constraints = [cp.matmul(A, x) <= 56, x<=1, x>=-1]
objective = cp.Maximize(x[0]+x[1])
# objective = cp.Maximize(0)
# objective = cp.Maximize(cp.sum(x, axis=0))
problem = cp.Problem(objective, constraints)

solution = problem.solve()
print(solution)
print(x.value.T)


b = np.array([56, 56])
x0_bounds = (-1, 1)
x1_bounds = (-1, 1)
c = [-1, -1]

res = linprog(c, A_ub=A, b_ub=b, bounds=[x0_bounds
        , x1_bounds])
print(res.x)