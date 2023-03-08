import cvxpy as cp
import numpy as np

# The analytical solution to Ax = b
A = np.array([[1,10],[2,21],[3,12]])
b = np.array([[2], [3], [4]])
# b = np.array([2, 3, 4])

x_projected = np.dot(np.linalg.pinv(A),b)
print("The analytical solution to Ax = b \n", x_projected)

# Solving as a QP problem 
x = cp.Variable((2, 1))
# x = cp.Variable(2)
I = np.identity(3)
obj = cp.Minimize(cp.quad_form(A@x-b, I) )
prob = cp.Problem(obj)
result = prob.solve()
print(" Solving as a QP problem: \n", x.value)

# # Adding some linear constraints
# constraints = [ 1 <= x[0], x[0] <= 2, x[1] <= 5,  2 <= x[1]]
# prob = cp.Problem(obj, constraints)
# result = prob.solve()
# print("Solving as a QP problem with constraints: \n", x.value)

# # Adding some linear constraints
# constraints = [ -3.9 <= x, x <= 3.9]
# prob = cp.Problem(obj, constraints)
# result = prob.solve()
# print("After constraint x on -3.9 <= x, x <= 3.9 \n", x.value)


