"""
author: Geesara Prathap, Peter Huang
email: ggeesara@gmail.com, hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
import mpl_toolkits.mplot3d as a3
import pylab as p
import mpl_toolkits.mplot3d.axes3d as p3
import scipy as sp

class QuadCopter:
    def __init__(self):
        self.arm_length = 0.8
        self.height = 0.05
        self.body_frame = np.array([[self.arm_length, 0, 0, 1],
                       [0, self.arm_length, 0, 1],
                       [-self.arm_length, 0, 0, 1],
                       [0, -self.arm_length, 0, 1],
                       [0, 0, 0, 1],
                       [0, 0, self.height, 1]])
        self.history = np.zeros((500,3))
        self.count = 0

    def world_frame(self, state, xx1):
        origin = state[0:3]
        theta = state[3]
        wHb = [[np.cos(theta), -np.sin(theta), 0, origin[0]], [np.sin(theta), np.cos(theta),0, origin[1]]
            , [0,0,1, origin[2]],[0, 0, 0, 1]]
        quadBodyFrame = self.body_frame.T
        quadWorldFrame = np.dot(np.array(wHb),np.array(quadBodyFrame))
        world_frame = quadWorldFrame[0:3]
        return world_frame, state, xx1
    
    def plot_quad_3d(self, get_world_frame, obs_map):
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1], projection='3d')
        for obs in obs_map:
            pose_ = (obs[0], obs[1], obs[2])
            size_ = (obs[3], obs[3], obs[3])
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
            x = obs[0] + obs[3]*np.cos(u)*np.sin(v)
            y = obs[1] + obs[3]*np.sin(u)*np.sin(v)
            z = obs[2] + obs[3]*np.cos(v)
            ax.plot_wireframe(x, y, z, color="r")

        ax.plot([], [], [], '-', c='cyan')[0]
        ax.plot([], [], [], '-', c='red')[0]
        ax.plot([], [], [], '-', c='blue', markevery=2)[0]
        ax.plot([], [], [], '.', c='red', markersize=4)[0]
        ax.plot([], [], [], '.', c='green', markersize=2)[0]
        ax.plot([], [], [], '.', c='blue', markersize=2)[0]
        self.set_limit((-6,6), (-6,6), (0,6))
        an = animation.FuncAnimation(fig,
                                    self.anim_callback,
                                    fargs=(get_world_frame,),
                                    init_func=None,
                                    frames=400, interval=10, blit=False)
        plt.show()


    def set_limit(self, x, y, z):
        ax = plt.gca()
        ax.set_xlim(x)
        ax.set_ylim(y)
        ax.set_zlim(z)

    def anim_callback(self, i, get_world_frame):
        frame, state, xx1 = get_world_frame(i)
        self.set_frame(frame, state, xx1)

    def set_frame(self, frame, state, xx1):
        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
        ax = plt.gca()

        h_t = 0.14
        w_t = 0.09
        x1 = state[0]
        y1 = state[1]
        z1 = state[2]
        th1 = state[3]
        
        x1_tri = [x1+h_t*np.cos(th1), x1+(w_t/2)*np.cos((np.pi/2)-th1), x1-(w_t/2)*np.cos((np.pi/2.0)-th1)]
        y1_tri = [y1+h_t*np.sin(th1), y1-(w_t/2)*np.sin((np.pi/2)-th1), y1+(w_t/2)*np.sin((np.pi/2.0)-th1)]
        z1_tri = [z1 + 0.01, z1+0.01, z1+0.01]
        
        verts = np.zeros([3,3])
        verts[:,0] = x1_tri
        verts[:,1] = y1_tri
        verts[:,2] = z1_tri
        tri = a3.art3d.Poly3DCollection([verts])
        tri.set_color(colors.rgb2hex(sp.rand(3)))
        tri.set_edgecolor('k')
        ax.add_collection3d(tri)

        lines = ax.get_lines()
        lines[-2].set_data(xx1[:,0], xx1[:,1])
        lines[-2].set_3d_properties(xx1[:,2])

        for line, line_data in zip(lines[:3], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)
 
        self.history[self.count] = frame[:,4]
        if self.count < np.size(self.history, 0) - 1:
            self.count += 1
        zline = self.history[:self.count,-1]
        xline = self.history[:self.count,0]
        yline = self.history[:self.count,1]
        lines[-1].set_data(xline, yline)
        lines[-1].set_3d_properties(zline)






  

    