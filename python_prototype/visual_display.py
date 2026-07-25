import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

from State_variable import RK4_new_state
from State_variable import get_dry_CoM_dist_centre as start_dist
from State_variable import body_to_ecif as b2eci
from State_variable import ecif_to_body

from Environment import get_true_altitude
from Environment import get_atmosphere

from PID_controller import PID_controller as pidc

# --- ROCKET SETTINGS ---

mfr = 14e3           # mass flow rate kg/s
m_rocket_dry = 150e3 # mass of rocket WITHOUT fuel, in kg
m_fuel_max = 2e6     # total mass of fuel ONLY
gimbal_max = 0.2618  # max thrust engine angle in positive direction
gimbal_min = -0.2618 # max thrust engine angle in negative direction (include negative sign)

# --- PID CONTROLLER SETTINGS ---

kp = 5.0 # proportional coefficient
ki = 0.0 # integral coefficient
kd = 10.0 # derivative coefficient

# --- TIME & RENDERING SETTINGS ---

max_desired_frames = 110 # how many frames you want the animation to run for
no_limit = True          # if you want program to run without end, no_limit = True, if False, animation will run for max_desired_frames number of frames
time_interval = 0.01     # the RK4 physics step, smaller more higher accuracy, but may slow running time (in seconds)
steps_per_frame = 50     # how many physics steps to calculate before drawing 1 frame, increase for larger jumps in time
current_time = 0.0       # Start exactly at 0
# (500 steps * 0.01s = 5 seconds of simulated flight per visual frame)

earth_rad_launch = 6378137.0 # radius of the earth at launch
dry_CoG_launch = np.array( [start_dist(earth_rad_launch), 0.0, 0.0] ) 

# --- MAX Q VALUE ---
# variables for calculation of point of maximum dynamic pressure (Q)
max_q = 0.0
max_q_altitude = 0.0
max_q_time = 0.0
q_displayed = False

# --- WIND SHEAR EVENT TRIGGER ---
wind_shear_trigger_time = 10                  # time wind shear event occurs
gust_duration = 5.0                           # how long the wind hits the rocket (seconds)
lateral_wind_speed_x = 0.0                    # 150 m/s is roughly a Category 5 Hurricane
lateral_wind_speed_y = 190.0 
lateral_wind_speed_z = -80.0     
v_wind_baseline = np.array([0.0, 15.0, 30.0]) # general wind speed

\

initial_state = np.array([
    dry_CoG_launch[0], dry_CoG_launch[1], dry_CoG_launch[2],   # position of rocket's dry CoG in the ecif, in meters
    0.0, 0.0, 0.0,                                             # velocity in ecif, in m/s
    1.0, 0.0, 0.0, 0.0,                                        # quaternion orientation
    0.0, 0.0, 0.0,                                             # angular velocity in rocket's body frame, in rad/s
    m_fuel_max + m_rocket_dry                                  # mass, in kg
])

\

current_state = initial_state.copy() # reasoning for the .copy is to avoid previous error where runtime was carried over by ide
current_time = -time_interval        # defined as such so the program starts at t = 0

\

def eci_to_ecef(r_eci, t): # r_eci is the vector position vector of the rocket in cartestian coords within the earth centered inertial frame

    """ Function converting a vector in the ecif to the eceff """

    omega_e = 7.2921159e-5  # angular velocity of earth rotation in rad/s
    theta = omega_e * t
    
    # Rotation matrix
    R = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])
    
    return R @ r_eci

\

# --- ANIMATION SETUP ---

# create figure display
fig_mc = plt.figure(figsize = (12, 6))
gs = gridspec.GridSpec(2, 3, width_ratios = [1.5, 1, 1]) # creates an n x m grid on the figure, width ratios make the left column larger than the right

# create axes for 3D interface, gs[:, 0] makes it span over all rows, but stays in first column
ax_3d_vis = fig_mc.add_subplot(gs[:, 0], projection = '3d')
ax_3d_vis.set_title('3D ECIF Trajectory')


# create axes for altitude vs time plot (top middle)
ax_alt = fig_mc.add_subplot(gs[0, 1])
ax_alt.set_title('Altitude vs Time')
ax_alt.set_ylabel('Altitude (m)')

# create axes for Mach vs time plot (bottom middle)
ax_mach = fig_mc.add_subplot(gs[1, 1])
ax_mach.set_title('Mach vs Time')
ax_mach.set_xlabel('Time (s)') # share axes of right side plots
ax_mach.set_ylabel('Mach')

# create axes for gimbal angle (bottom right)
ax_gim = fig_mc.add_subplot(gs[1, 2])
ax_gim.set_title('Gimbal angle vs Time')
ax_gim.set_xlabel('Time (s)')
ax_gim.set_ylabel('Angle (rad)')

# create axes for orientation display (top right)
ax_ori = fig_mc.add_subplot(gs[0, 2], projection = '3d') # attitude indicator

# set axes limits
ax_3d_vis.set_xlim([-1e7, 1e7])
ax_3d_vis.set_ylim([-1e7, 1e7])
ax_3d_vis.set_zlim([-1e7, 1e7])

# lists to hold the growing history of the rocket's flight
rocket_x, rocket_y, rocket_z, altitude_history, mach_history, time_history, pitch_history, yaw_history = [], [], [], [], [], [], [], []

# get initial values, transform if necessary, and add to lists
rocket_r_init = eci_to_ecef(initial_state[0:3], 0)
rocket_x.append(rocket_r_init[0])
rocket_y.append(rocket_r_init[1])
rocket_z.append(rocket_r_init[2])
initial_alt = get_true_altitude(initial_state[0:3])
altitude_history.append(initial_alt)
mach_history.append(0)
time_history.append(0)
pitch_history.append(0)
yaw_history.append(0)

# create line objects
trajectory_line, = ax_3d_vis.plot([], [], [], color = 'red', label = 'Trajectory')
current_pos_dot, = ax_3d_vis.plot([], [], [], 'o', color = 'orange', label = 'Current Pos')
mach_line, = ax_mach.plot([], [], color = 'green', label = 'Mach Number')
alt_line, = ax_alt.plot([], [], color = 'blue', label = 'Altitude')
pitch_line, = ax_gim.plot([], [], color = 'cyan', label = 'Pitch')
yaw_line, = ax_gim.plot([], [], color = 'olive', label = 'Yaw')

ax_3d_vis.legend()
ax_gim.legend()

# --- SIMULATION FUNCTIONS ---

is_paused = False
# define pitch and yaw pid controllers here, outside of update function so memory for integral controller is not wiped 
pitch_pid = pidc(kp, ki, kd, 0, gimbal_max, gimbal_min) 
yaw_pid = pidc(kp, ki, kd, 0, gimbal_max, gimbal_min)

def on_key_press(event):

    """ Listens for keyboard input on the Matplotlib figure, pressing 'enter' or spacebar toggles the pause state """

    global is_paused

    # check if the user pressed the Enter key or the Spacebar
    if event.key in ['enter', ' ']:
        is_paused = not is_paused

        if is_paused:
            print("\n[SIMULATION PAUSED] Press Enter or Space to resume...")
            ani.event_source.stop()  # freezes the animation loop

        else:
            print("\n[SIMULATION RESUMED]")
            ani.event_source.start() # unfreezes and continues the loop

# Connect the keypress event to your figure window
fig_mc.canvas.mpl_connect('key_press_event', on_key_press)

# --- THE INIT FUNCTION ---
def init():

    current_pos_dot.set_data([], [])
    current_pos_dot.set_3d_properties([]) # type: ignore

    trajectory_line.set_data([], [])
    trajectory_line.set_3d_properties([]) # type: ignore
    alt_line.set_data([], [])
    mach_line.set_data([], [])
    pitch_line.set_data([], [])
    yaw_line.set_data([], [])

    return trajectory_line, current_pos_dot, alt_line, mach_line, ax_ori, ax_gim

# --- THE UPDATE FUNCTION ---
def update(frame):

    """ Returns required values for all animations """

    global current_time, current_state, ani, is_paused, max_q, max_q_altitude, max_q_time, q_displayed   # Use global so they persist between frames

    if is_paused:
        return trajectory_line, current_pos_dot, alt_line, mach_line, ax_ori, pitch_line, yaw_line  # keep current frame frozen

    for _ in range(steps_per_frame):
        # step the physics engine forward
        current_time += time_interval

        # define variables for PID pre new state calculation
        current_position = current_state[0:3]
        ori = current_state[6:10] # quaternion orientation

        #  PID controller for pitch and yaw angles
        up_eci = current_position / np.linalg.norm(current_position)   # unit radial vector, in rocket's perspective this always points straight 'up', hence is 'target'
        up_body = ecif_to_body(up_eci, ori)                            #  convert target to rocket's body frame
        yaw_error = np.arctan2(up_body[1], up_body[0])                 # Deviation in the XY plane (v_y, v_x)
        pitch_error = np.arctan2(up_body[2], up_body[0])               # Deviation in the XZ plane (v_z, v_x)
        pitch_angle = pitch_pid.correction(pitch_error, time_interval)
        yaw_angle = yaw_pid.correction(yaw_error, time_interval)

        if wind_shear_trigger_time <= current_time <= (wind_shear_trigger_time + gust_duration) and wind_shear_trigger_time != 0:
            # Inject a violent wind coming from the East (X-axis)
            v_wind = np.array([lateral_wind_speed_x, lateral_wind_speed_y, lateral_wind_speed_z])

        else:
            # standard baseline wind 
            v_wind = v_wind_baseline

        # information for current frame
        current_state_info = RK4_new_state(current_time, current_state, time_interval, mfr, m_rocket_dry, m_fuel_max, v_wind, pitch_angle, yaw_angle)
        current_state = current_state_info[0]
        current_position = current_state[0:3]
        current_vel = current_state[3:6]
        current_vel_mag = np.linalg.norm(current_vel)
        alt = get_true_altitude(current_position)

        # --- MAX Q OUTPUT ---
        _, _, density = get_atmosphere(alt) # density for max Q calculations
        current_q = 0.5 * density * (current_vel_mag ** 2) # current Q value

        if current_q > max_q:
            max_q = current_q
            max_q_altitude = alt
            max_q_time = current_time

        # calculation of current speed and conversion to mach
        _, _, temperature = get_atmosphere(alt)
        temperature += 273.15 # convert temperature from celcius to kelvin

        if temperature < 0.1:
            current_vel_mach = 0 # this avoides the div by zero error for high altitudes causing temperature --> 0
        else:
            v_sound = np.sqrt( 1.4 * 287.05 * temperature ) # speed of sound varies with altitude
            current_vel_mach = current_vel_mag / v_sound    # coverting to mach number

        # --- GROUND COLLISION CHECK ---
        if alt <= 0.0 and current_time > 0:
            print(f"\n IMPACT DETECTED at t = {current_time:.1f}s. Stopping simulation.")
            print(f"\n Max Q is {max_q:.1f}Pa, which occurs at altitude {max_q_altitude:.1f}m, at time {max_q_time:.1f}s")
            ani.event_source.stop() # Freezes the animation loop

            return trajectory_line, current_pos_dot, alt_line, mach_line, ax_ori, pitch_line, yaw_line
    

    print(f" ------------------------------------------------------------------------------ \n Time = {current_time:.1f}s, Fuel mass = {current_state_info[1]}kg, Total mass = {current_state_info[2]}kg, Altitude = {current_state_info[3]:.1f}m") # mission control display
    if alt > 85000 and q_displayed == False: # out of atmosphere

        print(f" ------------------------------------------------------------------------------ \n Max Q is {max_q:.1f}Pa, which occurs at altitude {max_q_altitude:.1f}m, at time {max_q_time:.1f}s")
        q_displayed = True

    # orientation calculations
    up_eci = current_state[0:3] / np.linalg.norm(current_state[0:3]) # radial vector, in rocket's perspective this always points straight 'up'
    nose_ori_bf = np.array([1.0, 0.0, 0.0])                          # vector aligned with nose cone in rocket body frame
    nose_ori_eci = b2eci(nose_ori_bf, ori)                           # vector aligned with nose cone in ecif
    north_pole_eci = np.array([0.0, 0.0, 1.0])                       # north vector defined along z axis in ecif

    # if at north or south pole, nose cone vector is parallel to north vector in ecif, so cross product is 0, we check if this is the case and assign the y direction as east
    if np.allclose(np.abs(up_eci), north_pole_eci):
        east_eci = np.array([0.0, 1.0, 0.0])
    else:
        east_eci = np.cross(north_pole_eci, up_eci)
        east_eci = east_eci / np.linalg.norm(east_eci)

    north_local = np.cross(up_eci, east_eci)
    R_eci_to_local = np.vstack([east_eci, north_local, up_eci]) # stacked the three vectors, north, east and 'up', into a matrix which represents a basis

    up_local = R_eci_to_local @ up_eci         # convert 'up' vector from eci to new basis
    nose_local = R_eci_to_local @ nose_ori_eci # convert nose vector from eci to new basis

    # attitude display
    ax_ori.cla() # wipe previous time's arrow
    ax_ori.quiver( 
        0, 0, 0,
        up_local[0], up_local[1], up_local[2],
        color = 'green',
        linewidth = 3
    )
    ax_ori.quiver(
        0, 0, 0,
        nose_local[0], nose_local[1], nose_local[2],
        color = 'red',
        linewidth = 3
    )

    ax_ori.set_xlim([-1, 1])
    ax_ori.set_ylim([-1, 1])
    ax_ori.set_zlim([-1, 1])
    ax_ori.set_box_aspect([1, 1, 1])
    ax_ori.set_axis_off()
    ax_ori.set_title('Attitude Indicator')

    # convert new ECI position to ECEF
    current_rocket_r = eci_to_ecef(current_state[0:3], current_time)

    # append to history lists
    rocket_x.append(current_rocket_r[0])
    rocket_y.append(current_rocket_r[1])
    rocket_z.append(current_rocket_r[2])
    altitude_history.append(alt)
    mach_history.append(current_vel_mach)
    time_history.append(current_time)
    pitch_history.append(pitch_angle)
    yaw_history.append(yaw_angle)
    
    # update the line data on the plot
    trajectory_line.set_data(rocket_x, rocket_y)
    trajectory_line.set_3d_properties(rocket_z)  # type: ignore
    
    current_pos_dot.set_data([current_rocket_r[0]], [current_rocket_r[1]])
    current_pos_dot.set_3d_properties([current_rocket_r[2]])  # type: ignore

    alt_line.set_data(time_history, altitude_history)
    mach_line.set_data(time_history, mach_history)
    pitch_line.set_data(time_history, pitch_history)
    yaw_line.set_data(time_history, yaw_history)
    
    # set travelling axes for altitude and speed time plots

    ax_alt.set_xlim(0, max(5, max(time_history) * 1.1))
    ax_alt.set_ylim(0, max(1000, max(altitude_history) * 1.1)) # y from 0 to maximum altitude plus some leeway, with least being 1000m

    ax_mach.set_xlim(0, max(5, max(time_history) * 1.1))
    ax_mach.set_ylim(0, max(2, max(mach_history) * 1.1))

    ax_gim.set_xlim(max(5, max(time_history) * 0.1), max(5, max(time_history) * 1.1))
    ax_gim.set_ylim(gimbal_min * 1.1, gimbal_max * 1.1)

    # stops animation after a certain number of frames
    if frame >= max_desired_frames and no_limit == False:

        if "ani" in globals():
            ani.event_source.stop()

        return trajectory_line, current_pos_dot, alt_line, mach_line, ax_ori, pitch_line, yaw_line
    
    return trajectory_line, current_pos_dot, alt_line, mach_line, ax_ori, pitch_line, yaw_line

# --- EARTH REPERSENTATION ---

# define separate equatorial and polar radii (WGS 84 ellipsoid standards)
R_eq = 6378137.0   # equatorial radius in meters
R_pol = 6356752.3  # polar radius in meters

u = np.linspace(0, 2 * np.pi, 50)
v = np.linspace(0, np.pi, 50)

# Scale x and y by the equatorial radius
x = R_eq * np.outer(np.cos(u), np.sin(v))
y = R_eq * np.outer(np.sin(u), np.sin(v))

# Scale z by the slightly smaller polar radius
z = R_pol * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface as a blue translucent sphere
ax_3d_vis.plot_surface(x, y, z, color='blue', alpha=0.3)

# Force the axes to maintain a 1:1:1 ratio
ax_3d_vis.set_box_aspect([1, 1, 1])

# --- START THE ANIMATION ENGINE ---

ani = animation.FuncAnimation(
    fig = fig_mc,           # The figure canvas to draw on
    func = update,          # The function to call every frame
    init_func = init,       # The setup function
    frames = 10000,         # Number of frames to run (or a generator)
    interval = 50,          # Milliseconds to wait between frames (50ms = 20 fps)
    blit = False            # Set to False for 3D plots (Matplotlib 3D doesn't support true blitting)
)


plt.tight_layout()

# Save the running animation directly as a GIF
#ani.save(
#    "windshear_stabilization.gif",
#    writer = "pillow",
#    fps  = 20,
#    dpi = 100
#)

plt.show()

