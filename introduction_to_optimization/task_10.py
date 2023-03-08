import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog

def plot_line(slope, intercept):
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = intercept + slope * x_vals
    start, end = axes.get_xlim()
    axes.xaxis.set_ticks(np.arange(start, end, 0.5))
    start, end = axes.get_ylim()
    axes.yaxis.set_ticks(np.arange(start, end, 0.1))
    plt.plot(x_vals, y_vals, '-')

plt.ylim((-0.8, 0.8))
plt.xlim((-5,5))

K = -1*np.array([[1.19, 7.88]])
A = np.array([[1.1, 2], [0, 0.95]])
B = np.array([[0], [0.0787]])

N = 20
for i in range(1, N):
    constraints_set_upper = []
    obj = np.dot(K[0], np.linalg.matrix_power(A+np.dot(B, K), i+1))
    
    for j in range(0, i):
        ex = np.linalg.matrix_power(A+np.dot(B, K), j)
        index = np.dot(K, ex)[0]
        constraints_set_upper.append(index)
        
    constraints_set_upper = np.array(constraints_set_upper)
    constraints_set_lower = -1.0*constraints_set_upper
    constraints_set = np.vstack((constraints_set_upper, constraints_set_lower))
    lhs_ineq_1 = np.ones((1,constraints_set_upper.shape[0]))
    lhs_ineq_2 = np.ones((1,constraints_set_upper.shape[0]))
    control_bound = np.transpose(np.hstack((lhs_ineq_1, lhs_ineq_2)))
    opt = linprog(c=obj, A_ub=constraints_set, b_ub=control_bound
                                        , bounds=[(None, None), (None, None)])
    c = np.dot(opt.x, obj)
    a_b = obj
    plot_line((-1*a_b[0])/a_b[1], c/a_b[1])
    plot_line((-1*a_b[0])/a_b[1], -c/a_b[1])

plt.grid(True)
plt.show()