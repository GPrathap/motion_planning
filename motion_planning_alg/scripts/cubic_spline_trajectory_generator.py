import math
import numpy as np
import bisect
import matplotlib.pyplot as plt

class Spline1D:
    def __init__(self, t, y):
        self.a, self.b, self.c, self.d = [], [], [], []

        self.t = t
        self.y = y

        self.n = len(t)
        h = np.diff(t)

        self.a = [iy for iy in y]
        
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)

        for i in range(self.n - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def cal(self, t):
        if t < self.t[0]:
            return None
        elif t > self.t[-1]:
            return None
        i = self.__search_index(t)
        dx = t - self.t[i]
        result = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def cald(self, t):
        if t < self.t[0]:
            return None
        elif t > self.t[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.t[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def caldd(self, t):
        if t < self.t[0]:
            return None
        elif t > self.t[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.t[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        return bisect.bisect(self.t, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.n, self.n))
        A[0, 0] = 1.0
        for i in range(self.n - 1):
            if i != (self.n - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.n - 1, self.n - 2] = 0.0
        A[self.n - 1, self.n - 1] = 1.0
        return A

    def __calc_B(self, h):
        B = np.zeros(self.n)
        for i in range(self.n - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline1D(self.s, x)
        self.sy = Spline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.cal(s)
        y = self.sy.cal(s)
        return x, y

    def calc_curvature(self, s):
        dx = self.sx.cald(s)
        ddx = self.sx.caldd(s)
        dy = self.sy.cald(s)
        ddy = self.sy.caldd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k
    
    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def calc_yaw(self, s):
        dx = self.sx.cald(s)
        dy = self.sy.cald(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calculate_reference_trajectory(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    return np.array(rx), np.array(ry), np.array(ryaw), np.array(rk), np.array(s)

def main():
    
    x = [50, 59, 50, 57, 40, 40]
    y = [25, 12, 10, 2, 4, 14]
    ds = 0.1
    
    rx, ry, ryaw, rk, s = calculate_reference_trajectory(x, y, ds=ds)
    
    selected_pathx = rx[np.where((s>=40) & (s<=45))]
    selected_pathy = ry[np.where((s>=40) & (s<=45))]
    
    flg, ax = plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(rx, ry, "-r", label="spline")
    plt.plot(selected_pathx, selected_pathy, "-b", label="selected")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    
    
    
    
    flg, ax = plt.subplots(1)
    plt.plot(s, [math.degrees(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    flg, ax = plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    main()