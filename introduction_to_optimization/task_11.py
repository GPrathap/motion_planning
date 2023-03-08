import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog


def plot_line(slope, intercept, range_xval=(-8, 8)):
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = intercept + slope * x_vals
    start, end = axes.get_xlim()
    axes.xaxis.set_ticks(np.arange(start, end, 0.5))
    start, end = axes.get_ylim()
    axes.yaxis.set_ticks(np.arange(start, end, 0.1))
    plt.plot(x_vals, y_vals, '-')

v = 10
F = np.array([[0, 1.0/8.0], [1.0/8.0, 0], [0, -1*1.0/8.0], [-1.0/8.0, 0], [0, 0], [0, 0]])
G = np.array([[0,0,0,0,1.0, -1.0]]).transpose()

A = np.array([[1.1, 2], [0, 0.95]])
B = np.array([[0], [0.0787]])
C = np.array([-1, 1])
K = -1*np.array([[1.19, 7.88]])



