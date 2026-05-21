"""
This file contains the main code for defining the dynamics of a 2D rimless wheel moving down a slope
"""

# Dependencies
import math
import scipy as sp
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# # Define paramaters
# Slope
slope_angle = 30  # degrees
slope_angle = math.radians(slope_angle)  # angle in radians

# Wheel
l = 0.15   # leg length in m
m = 1  # mass in kg
I = 0.1  # moment of inertia about center of mass/center of wheel in kgm^2
n = 8 # spokes

J = I/(2*m*l**2)

lam = 1/(2*J+1)
print(lam)

collision_angle = np.pi/n + slope_angle

# initial conditions
init_ang = 0.2
init_vel = 1
init_con = np.array([init_ang, init_vel])

# print(len(init_con))

def dydt(t, y):
    dydt = np.array([y[1], lam**2*np.sin(y[0])])
    return dydt

def collision(t, y):
    return y[0] - collision_angle

collision.terminal = True

solution = solve_ivp(dydt, [0, 10], init_con, max_step = 0.001, events=collision)

y_sol = solution.y

print(solution.y.shape)

print(y_sol[1, :])

print(solution.t_events)

plt.plot(solution.t, solution.y[0])
plt.plot(solution.t, collision_angle*np.ones(solution.y.shape[1]))
plt.show()




