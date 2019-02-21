from __future__ import print_function
import numpy as np
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from plotWindow import plotWindow
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=150)
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

LOG_WIDTH = 1 + 13 + 4 + 13 + 4 + 1

data = np.reshape(np.fromfile("/tmp/LQR_controller_mocap.bin", dtype=np.float64), (-1, LOG_WIDTH)).T
t = data[0,:]
x = data[1:14,:]
u = data[14:18,:]
xc = data[18:31, :]
ur = data[31:35, :]

compute_time = data[35, :]
print("Mean compute time")
print(np.mean(compute_time), "micro seconds")
print("STD")
print(np.std(compute_time), "micro seconds")

pw = plotWindow()

xlabel = [r'$p_x$', r'$p_y$', r'$p_z$',
          r'$q_w$', r'$q_x$', r'$q_y$', r'$q_z$',
          r'$v_x$', r'$v_y$', r'$v_z$',
          r'$\omega_x$', r'$\omega_y$', r'$\omega_z$']

ulabel = [r'$F$',
          r'$\tau_x$', r'$\tau_y$', r'$\tau_z$']

f = plt.figure(dpi=150)
plt.plot()
plt.plot(x[1,:], x[0,:], label="x")
plt.plot(xc[1,:], xc[0,:], label=r"$x_c$")
pw.addPlot("2D Position", f)

f = plt.figure(dpi=150)
plt.plot()
for i in range(3):
    plt.suptitle("Position")
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i,:], label="x")
    plt.plot(t, xc[i,:], label=r"$x_c$")
    plt.ylabel(xlabel[i])
    if i == 0:
        plt.legend()
pw.addPlot("Position", f)

f = plt.figure(dpi=150)
plt.plot()
for i in range(4):
    plt.suptitle("Attitude")
    plt.subplot(4, 1, i+1)
    plt.plot(t, x[i+3,:], label="x")
    plt.plot(t, xc[i+3,:], label=r"$x_c$")
    plt.ylabel(xlabel[i+3])
    if i == 0:
        plt.legend()
pw.addPlot("Attitude", f)

f = plt.figure(dpi=150)
plt.plot()
for i in range(3):
    plt.suptitle("Velocity")
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i+7,:], label="x")
    plt.plot(t, xc[i+7,:], label=r"$x_c$")
    plt.ylabel(xlabel[i+7])
    if i == 0:
        plt.legend()
pw.addPlot("Velocity", f)

f = plt.figure(dpi=150)
plt.plot()
for i in range(3):
    plt.suptitle("Omega")
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i+10,:], label="x")
    plt.plot(t, xc[i+10,:], label=r"$x_c$")
    # plt.ylabel(xlabel[i+10])
    if i == 0:
        plt.legend()
pw.addPlot("Omega", f)

pw.show()


