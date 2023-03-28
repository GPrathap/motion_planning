import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math

# sudo apt install python3-tk -y  

t = np.arange(0, 2*np.pi, 0.1, dtype=float)

# x_t = np.sin(t)
# y_t = np.cos(t)

x_t = t
y_t = t

fig = plt.figure(1)
plt.plot(x_t, y_t, label="r(t)")
plt.show()