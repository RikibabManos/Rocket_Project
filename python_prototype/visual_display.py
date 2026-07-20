import math
import numpy as np
import matplotlib.pyplot as plt
from State_variable import RK4_new_state
from matplotlib.animation import FuncAnimation

# initial parameters

mfr = 25000 # mass flow rate kg/s
m_rocket_dry = 3.0e4
m_fuel_max = 2.0e6

initial_state = np.array([
    6378137.0, 0.0, 0.0,   # 0:2 - Position (ECI meters)
    0.0, 0.0, 0.0,         # 3:5 - Velocity (ECI m/s)
    1.0, 0.0, 0.0, 0.0,    # 6:9 - Quaternion (Orientation)
    0.0, 0.0, 0.0,         # 10:12 - Angular Velocity (Body rad/s)
    m_fuel_max + m_rocket_dry   # 13 - Mass (kg)
])

def eci_to_ecef(r_eci, t): # r_eci is the vector position vector of the rocket in cartestian coords within the earth centered inertial frame
    omega_e = 7.2921159e-5  # angular velocity off earth rotation in rad/s
    theta = omega_e * t
    
    # Rotation matrix
    R = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    
    return R @ r_eci  # Matrix multiplication

\

# 1. Initialize 3D plot
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

# 2. Draw the Earth (Simplified Sphere)
#R_earth = 6378137.0 # Earth radius in meters
#u = np.linspace(0, 2 * np.pi, 50)
#v = np.linspace(0, np.pi, 50)
#x = R_earth * np.outer(np.cos(u), np.sin(v))
#y = R_earth * np.outer(np.sin(u), np.sin(v))
#z = R_earth * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface as a blue translucent sphere
#ax.plot_surface(x, y, z, color='blue', alpha=0.3)

# 3. Plot the Rocket Trajectory

# list of rocket coords imn ECEF frame
# at time t = 0

# Initialize empty lists
#rocket_x = []
#rocket_y = []
#rocket_z = []

# Get initial position and transform it
#rocket_r_init = eci_to_ecef(initial_state[0:3], 0)

# Append initial position to lists
#rocket_x.append(rocket_r_init[0])
#rocket_y.append(rocket_r_init[1])
#rocket_z.append(rocket_r_init[2])

# generate points in t = 0.5s intervals

current_time = 0
current_state = initial_state
time_interval = 0.5

#for i in range(1000):
#    current_time += time_interval
#    current_state = RK4_new_state(current_time, current_state, time_interval, mfr, m_rocket_dry)
#    current_rocket_r = eci_to_ecef(current_state[0:3], current_time)
#    rocket_x.append(current_rocket_r[0])
#    rocket_y.append(current_rocket_r[1])
#    rocket_z.append(current_rocket_r[2])


#ax.plot(rocket_x, rocket_y, rocket_z, color='red', label='Trajectory')
#ax.scatter(rocket_x[-1], rocket_y[-1], rocket_z[-1], color='orange', s=50, label='Current Pos')

#ax.legend()


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # REQUIRED for older versions

# 1. THE SETUP
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set your axes limits (e.g., Earth radii)
ax.set_xlim([-1e7, 1e7])
ax.set_ylim([-1e7, 1e7])
ax.set_zlim([-1e7, 1e7])

# Lists to hold the growing history of the rocket's flight
rocket_x, rocket_y, rocket_z = [], [], []
# Get initial position, transform it, and add to lists
rocket_r_init = eci_to_ecef(initial_state[0:3], 0)
rocket_x.append(rocket_r_init[0])
rocket_y.append(rocket_r_init[1])
rocket_z.append(rocket_r_init[2])

# Create the line objects
trajectory_line, = ax.plot([], [], [], color='red', label='Trajectory')
current_pos_dot, = ax.plot([], [], [], 'o', color='orange', label='Current Pos')
ax.legend()

# 2. THE INIT FUNCTION
def init():
    
    trajectory_line.set_data([], [])
    trajectory_line.set_3d_properties([]) # type: ignore
    
    current_pos_dot.set_data([], [])
    current_pos_dot.set_3d_properties([]) # type: ignore
    return trajectory_line, current_pos_dot

# 3. THE UPDATE FUNCTION
# 'frame' is an auto-incrementing integer (0, 1, 2, 3...) passed by FuncAnimation
def update(frame):
    """Calculates the next RK4 step and updates the 3D plot."""
    global current_time, current_state  # Use global so they persist between frames
    
    # Step the physics engine forward
    current_time += time_interval
    current_state = RK4_new_state(current_time, current_state, time_interval, mfr, m_rocket_dry)
    
    # Convert new ECI position to ECEF
    current_rocket_r = eci_to_ecef(current_state[0:3], current_time)
    
    # Append to history lists
    rocket_x.append(current_rocket_r[0])
    rocket_y.append(current_rocket_r[1])
    rocket_z.append(current_rocket_r[2])
    
    # Update the line data on the plot
    trajectory_line.set_data(rocket_x, rocket_y)
    trajectory_line.set_3d_properties(rocket_z)  # type: ignore
    
    current_pos_dot.set_data([current_rocket_r[0]], [current_rocket_r[1]])
    current_pos_dot.set_3d_properties([current_rocket_r[2]])  # type: ignore
    
    return trajectory_line, current_pos_dot

R_earth = 6378137.0 # Earth radius in meters
u = np.linspace(0, 2 * np.pi, 50)
v = np.linspace(0, np.pi, 50)
x = R_earth * np.outer(np.cos(u), np.sin(v))
y = R_earth * np.outer(np.sin(u), np.sin(v))
z = R_earth * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface as a blue translucent sphere
ax.plot_surface(x, y, z, color='blue', alpha=0.3)

# 4. START THE ANIMATION ENGINE
ani = animation.FuncAnimation(
    fig=fig,              # The figure canvas to draw on
    func=update,          # The function to call every frame
    init_func=init,       # The setup function
    frames=10000,          # Number of frames to run (or a generator)
    interval=50,          # Milliseconds to wait between frames (50ms = 20 fps)
    blit=False            # Set to False for 3D plots (Matplotlib 3D doesn't support true blitting)
)

plt.show()