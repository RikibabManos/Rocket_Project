import numpy as np
import pandas as pd

from Environment import get_atmosphere
from Environment import get_true_altitude
from Environment import grav_acc

from Coeff_fitting import get_coeff_value

from Centres_of import get_CoG_fuel_bf
from Centres_of import get_CoG_total
from Centres_of import get_CoP

\

# --- .CVS FILES ---

# first .cvs file is for Forebody (cd) and Normal (cn) axial coefficients at certain Mach numbers
# the .cvs file must statisfy following conditions
# a header row (that will be ingnored)
# the columns must be arranged in the following order: mach number, cd values, cn values
# note that cn values must be per radian (based off angle of attack)

df = pd.read_csv(
    r"C:\Codes\Rocket_project\coeff_cvs\approx_saturn_v_coeff.csv"
    )

aero_table = df.to_numpy()
mach_numbers_aero = aero_table[:, 0]  
cd_values = aero_table[:, 1]     
cn_values = aero_table[:, 2]     

# second .csv file containing Centre of Pressure (CoP) data for a given rocket
# must be arranged in following columns: Mach number, CoP @ alpha rad, CoP @ beta rad...
# very important that the headings for the different aoa are the aoa values in radians

df = pd.read_csv(
    r"C:\Codes\Rocket_project\coeff_cvs\appox_saturn_v_CoP_lookup.csv",
    index_col = 0
    ) 

mach_numbers_CoP = df.index.to_numpy()           # mach numbers data into an array without heading
aoa_values = df.columns.astype(float).to_numpy() # aoa values extracted from headers 
coeff_matrix = df.to_numpy()                     # CoP values 

\

# --- CONSTANTS ---

# General
A_EQ = 6378137.0       # Equatorial radius, WGS84 Earth reference ellipsoid constants (in meters)
B_POL = 6356752.314245 # Polar radius, WGS84 Earth reference ellipsoid constants (in meters)
MU = 3.986004418e14    # Precomputed Earth Standard Gravitational Parameter (m^3/s^2)
P_ATM = 101.325        # Kilo-Pascals (standard sea level pressure)
G0 = 9.80665           # gravitational acceleration on earth's surface

# Rocket Specific

# Specific impules (Example used: SpaceX Merlin 1D Engine)
ISP_SL = 282.0   # seconds, at sea level
ISP_VAC = 311.0  # seconds, in a vacuum

d = 10.06                                    # max cone diameter, in meters (Example used: Saturn V)
total_rocket_height = 110.6                  # bottom to tip height of rocket, in meters (Example used: Saturn V)
fuel_tank_height = 32.6                      # total length of fuel tank, in meters (Example used: Saturn V)
bottom_to_tank = 5.94                        # approximate distance from bottom of rocket to bottom of fuel tank, in meters (Example used: Saturn V)
thruster_height = np.array([5.92, 0.0, 0.0]) # distance from bottom of rocket to fuel exit, in rocket body frame (Example used: Saturn V)
dry_CoG = np.array( [41.0, 0.0, 0.0] )       # approcimate CoG distance vector from bottom of dry rocket, in meters, in rocket bf (Example used: Saturn V)
aoa_initial = 0.0                            # angle of attack on launchpad, in radians
CSA = d **2 * np.pi * .25                    # based off given diameter

\

def get_dry_CoM_dist_centre(distance):
    
    """ Function returning the distance of the dry CoG of the rocket from the earth's centre """
    
    return distance + np.linalg.norm(dry_CoG) 

def get_current_isp(altitude):

    _, pressure, _ = get_atmosphere(altitude) # in kPa
    
    # Interpolate I_sp based on the pressure ratio assuming linear model
    current_isp = ISP_VAC + (ISP_SL - ISP_VAC) * (pressure / P_ATM) 
    
    return current_isp

def get_current_density(altitude):

    density, _, _ = get_atmosphere(altitude) # in kg/ cu m

    return density

def body_to_ecif(body_vector, orientation): # note orientation must be in quaternion form

    """ Function returns a vector in the ecif from the rockets body frame """

    orientation = orientation / np.linalg.norm(orientation) # make sure is normalised (avoid float point errors)
    q0 = orientation[0]
    q1 = orientation[1]
    q2 = orientation[2]
    q3 = orientation[3]

    # transformation matrix
    L = np.array([
        [ ( 1 - ( 2 * ( (q2 ** 2) + (q3 ** 2) ) ) ), 2 * ( (q1 * q2) - (q0 * q3) ), 2 * ( (q1 * q3) + (q0 * q2) ) ],
        [ 2 * ( (q1 * q2) + (q0 * q3) ), ( 1 - ( 2 * ( (q1 ** 2) + (q3 ** 2) ) ) ), 2 * ( (q2 * q3) - (q0 * q1) ) ],
        [ 2 * ( (q1 * q3) - (q0 * q2) ), 2 * ( (q2 * q3) + (q0 * q1) ), ( 1 - ( 2 * ( (q1 ** 2) + (q2 ** 2) ) ) ) ]
    ])

    new_frame = L @ body_vector 

    return new_frame

def ecif_to_body(eci_vector, orientation):
    """ Function returns a vector in the ecif from the rockets body frame """

    orientation = orientation / np.linalg.norm(orientation) # make sure is normalised (float point errors)
    q0 = orientation[0]
    q1 = orientation[1]
    q2 = orientation[2]
    q3 = orientation[3]

    # transformation matrix
    L = np.array([
        [ ( 1 - ( 2 * ( (q2 ** 2) + (q3 ** 2) ) ) ), 2 * ( (q1 * q2) - (q0 * q3) ), 2 * ( (q1 * q3) + (q0 * q2) ) ],
        [ 2 * ( (q1 * q2) + (q0 * q3) ), ( 1 - ( 2 * ( (q1 ** 2) + (q3 ** 2) ) ) ), 2 * ( (q2 * q3) - (q0 * q1) ) ],
        [ 2 * ( (q1 * q3) - (q0 * q2) ), 2 * ( (q2 * q3) + (q0 * q1) ), ( 1 - ( 2 * ( (q1 ** 2) + (q2 ** 2) ) ) ) ]
    ])

    new_frame = np.transpose(L) @ eci_vector

    return new_frame

def get_MoI(dry_mass, fuel_mass, diameter, rocket_height, fuel_height, total_CoG, dry_CoG, fuel_CoG): # note CoGs must be vector taken from the bottom of the rocket, in the bf
    
    """ Function returning the MoI tensor along its principle axes at a given mass of fuel about the total CoG"""

    r = diameter / 2 # radius

    MoI_tensor_dry = np.array([ # note that this is at the centre of gravity of the dry mass
    [  .5 * dry_mass * ( r ** 2 ) , 0, 0 ],
    [ 0,  ( 1 / 12) * dry_mass * ( (3 * (r**2)) + (rocket_height ** 2) ) , 0 ],
    [ 0, 0,  ( 1 / 12) * dry_mass * ( (3 * (r**2)) + (rocket_height ** 2) )  ]
    ])

    MoI_tensor_fuel = np.array([ # note that this is at the centre of gravity for the fuel
    [  .5 * fuel_mass * ( r ** 2 ) , 0, 0 ],
    [ 0,  ( 1 / 12) * fuel_mass * ( (3 * (r**2)) + (fuel_height ** 2) ) , 0 ],
    [ 0, 0,  ( 1 / 12) * fuel_mass * ( (3 * (r**2)) + (fuel_height ** 2) )  ]
    ])

    # now use parallel axis theorem to get MoI about the total CoG of the rocket

    total_to_dry_CoG = dry_CoG - total_CoG
    total_to_fuel_CoG = fuel_CoG - total_CoG
    I = np.array([ # 3x3 identity matrix
        [ 1, 0, 0 ],
        [ 0, 1, 0 ],
        [ 0, 0, 1 ]
    ])

    MoI_tensor_dry_shifted = MoI_tensor_dry + ( dry_mass * ( ( np.dot(total_to_dry_CoG, total_to_dry_CoG) * I ) - np.outer(total_to_dry_CoG, total_to_dry_CoG)) )
    MoI_tensor_fuel_shifted = MoI_tensor_fuel + ( fuel_mass * ( ( np.dot(total_to_fuel_CoG, total_to_fuel_CoG) * I ) - np.outer(total_to_fuel_CoG, total_to_fuel_CoG)) )

    MoI_tensor_total = MoI_tensor_dry_shifted + MoI_tensor_fuel_shifted # total MoI about instantaneous centre of mass
    
    return MoI_tensor_total

\

# --------------------------------------------------------
# ------------- THE CORE DERIVATIVE FUNCTION -------------
# --------------------------------------------------------

def get_derivatives(t, state, mfr, dry_mass, fuel_mass_max, v_wind, pitch_angle, yaw_angle):

    """ Takes the current state array and returns the rates of change """

    # unpack the state vector
    position = state[0:3] # this is the instantaneous position of the total CoG in the ecif
    velocity = state[3:6] # of total CoG in ecif
    q = state[6:10]       # q[0] is scalar, q[1:4] are vector
    omega = state[10:13]  # angular velocity
    m = state[13]         # mass

    fuel_mass = m - dry_mass
    actual_fuel_length = fuel_tank_height * (fuel_mass / fuel_mass_max) # the physical length of the fuel column currently in the tank

    # initialize the array that will hold our output derivatives
    state_dot = np.zeros(14)
    
    # --- TRANSLATIONAL KINEMATICS ---

    # the derivative of position is the current velocity
    state_dot[0:3] = velocity

    altitude = get_true_altitude(position)
    isp = get_current_isp(altitude)

    body_long_axis = np.array([1, 0, 0]) 
    rocket_long_axis_direc_eci = body_to_ecif(body_long_axis, q) # long axis direction of rocket converted to the ecif 

    # thrust vector analysis
    thrust_direc_bf = np.array([ ( np.cos(pitch_angle) * np.cos(yaw_angle) ), ( np.sin(yaw_angle) ), ( np.sin(pitch_angle) * np.cos(yaw_angle) ) ])
    thrust_direc_bf = thrust_direc_bf / np.linalg.norm(thrust_direc_bf) # normalise to remove effect float point errors
    thrust_direc_eci = body_to_ecif(thrust_direc_bf, q)
    thrust_mag = mfr * isp * G0
    thrust_force_eci = thrust_mag * thrust_direc_eci
    
    # gravitational force in ecif
    grav_force_eci = grav_acc(position) * m # inherently has a negative direction
    
    # aerodynamic forces

    altitude_const = .5 * get_current_density(altitude) * CSA # constant common to Lift and Drag force calculations

    v_rel = velocity - v_wind # velocity of wind relative to rocket
    v_rel_mag = np.linalg.norm(v_rel)

    if v_rel_mag > 0: # to avoid any div-by-zero errors (specifically at start)

        v_rel_direc = v_rel / v_rel_mag
        _, _, temperature =  get_atmosphere(altitude)
        temperature += 273.15 # convert temperature from celcius to kelvin

        if temperature < 0.1:
            v_rel_mag_mach = 0 # this avoides the div by zero error for high altitudes causing temperature --> 0
        else:
            v_sound = np.sqrt( 1.4 * 287.05 * temperature ) # speed of sound varies with altitude
            v_rel_mag_mach = v_rel_mag / v_sound            # coverting to mach number

        drag_force_eci = -altitude_const * v_rel_mag * v_rel * get_coeff_value(v_rel_mag_mach, mach_numbers_aero, cd_values) 

        cos_aoa = np.clip( np.dot(rocket_long_axis_direc_eci, v_rel_direc), -1.0, 1.0 ) # np.clip used so that no floating point errors from python cause value to have a mag > 1.0
        aoa = np.arccos(cos_aoa)                                                        # note np.arccos gives unit in radians
        aoa = np.clip(aoa, 0.0, np.pi / 2)                                              # Prevent aoa from exceeding the maximum bounds of 0 to 90 degree lookup table

        lift_force_mag = altitude_const * (v_rel_mag ** 2) * ( get_coeff_value(v_rel_mag_mach, mach_numbers_aero, cn_values) * aoa) # multiply by aoa as cn is given as a gradient per radian
        lift_force_resolved = rocket_long_axis_direc_eci - ( cos_aoa * v_rel_direc)                                                 # this is the vector that points in the direction of the lift force, but it is NOT normalised
        lift_force_resolved_mag = np.linalg.norm(lift_force_resolved)

        if lift_force_resolved_mag < 1e-8: # if rocket orientated directly up (relative to earth), lift force is zero, have small value to avoid float point errors
            lift_force_eci = np.zeros(3)

        else:
            lift_force_resolved_norm = lift_force_resolved / lift_force_resolved_mag
            lift_force_eci = lift_force_mag * lift_force_resolved_norm

    else:
        lift_force_eci = np.zeros(3)
        drag_force_eci = np.zeros(3)
        v_rel_mag_mach = 0.0         # velocity at launch = 0
        aoa = aoa_initial            # aoa at launch

    # Net Force and Acceleration
    net_force_eci = thrust_force_eci + grav_force_eci + drag_force_eci + lift_force_eci # Newton's 2nd Law, can be used directly as vectors are in an inertial frame
    acceleration_eci = net_force_eci / m
    
    # The derivative of velocity is acceleration
    state_dot[3:6] = acceleration_eci
    
    # --- ROTATIONAL KINEMATICS ---

    # firstly, calculate moment arms
    
    CoP = get_CoP(v_rel_mag_mach, aoa, mach_numbers_CoP, aoa_values, coeff_matrix, total_rocket_height) # in rocket bf from bottom
    CoG_fuel_bf = get_CoG_fuel_bf(fuel_tank_height, bottom_to_tank, fuel_mass, fuel_mass_max)
    total_CoG = get_CoG_total(dry_mass, dry_CoG, fuel_mass, CoG_fuel_bf)

    lever_arm_bf = CoP - total_CoG
    thrust_lever_bf = thruster_height - total_CoG
    # convert aerodynamic forces from ecif to rocket's bf, makes calculations easier as princlible axes can be used in bf

    drag_force_bf = ecif_to_body(drag_force_eci, q)
    lift_force_bf = ecif_to_body(lift_force_eci, q)

    # take anticlockwise as positive

    # torques
    lift_torque = np.cross(lever_arm_bf, lift_force_bf)
    drag_torque = np.cross(lever_arm_bf, drag_force_bf)
    thrust_torque = np.cross(thrust_lever_bf, (thrust_mag * thrust_direc_bf) )

    # resultant torque analysis
    MoI = get_MoI(dry_mass, fuel_mass, d, total_rocket_height, actual_fuel_length, total_CoG, dry_CoG, CoG_fuel_bf) # in rocket bf
    net_torque_bf = lift_torque + drag_torque + thrust_torque
    ang_momentum = MoI @ omega
    gyro_term = np.cross(omega, (ang_momentum)) # gyroscopic coupling term required to solve Euler's equations in non inertial rotating frame
    ang_acc_vec = np.linalg.solve(MoI, net_torque_bf - gyro_term)

    state_dot[10:13] = ang_acc_vec # adjust to state vector 

    # angular velocity components
    wx = omega[0]
    wy = omega[1]
    wz = omega[2]

    omega_matrix = np.array([ # this is the expression for omega as a quternion
        [0, -wx, -wy, -wz],
        [wx, 0, wz, -wy],
        [wy, -wz, 0, wx],
        [wz, wy, -wx, 0]
    ])

    state_dot[6:10] = .5 * ( omega_matrix @ q ) # quaternion derivative

    # --- MASS KINEMATICS ---
    # The derivative of mass is the negative mass flow rate
    state_dot[13] = -mfr 
    
    return state_dot

# --------------------------------------------------------
# ------------------ THE RK4 INTEGRATOR ------------------
# --------------------------------------------------------

def RK4_new_state(t, state, dt, mfr, dry_mass, fuel_max, v_wind, pitch, yaw): 

    # check how much fuel is left
    fuel_mass_remaining = state[13] - dry_mass

    # check how much fuel the engine can burn this frame based off mfr
    fuel_needed_this_step = mfr * dt

    # calculate adjusted mass flow rate
    if fuel_mass_remaining <= 0.0:
        # fuel tank is empty, shut the engine down
        mfr_adjusted = 0.0
        
    elif fuel_mass_remaining < fuel_needed_this_step:
        # not enough fuel for a full time step. 
        # set mfr so we burn EXACTLY what is left over the duration of 'dt'
        mfr_adjusted = fuel_mass_remaining / dt
        
    else:
        # plenty of fuel, run at normal throttle
        mfr_adjusted = mfr

    # --- MAIN RK4 ---
    k1 = get_derivatives(t, state, mfr_adjusted, dry_mass, fuel_max, v_wind, pitch, yaw)
    k2 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k1) / 2) ), mfr_adjusted, dry_mass, fuel_max, v_wind, pitch, yaw )
    k3 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k2) / 2) ), mfr_adjusted, dry_mass, fuel_max, v_wind, pitch, yaw )
    k4 = get_derivatives( t + dt , ( state +  (dt * k3) ), mfr_adjusted, dry_mass, fuel_max, v_wind, pitch, yaw )

    new_state = state + ( ( dt / 6 ) * ( k1 + ( 2 * k2 ) + ( 2 * k3 ) + k4 ) )

    fuel_mass_remaining = new_state[13] - dry_mass # for mission control
    altitude = get_true_altitude(new_state[0:3])

    # we must normalise the quaternion magnitude so that it represents a true rotation only

    quaternion = new_state[6:10]
    magnitude = np.linalg.norm(quaternion)

    # selection to notify of any div by zero errors, clearly code is buggy if mag = 0
    if magnitude == 0:
        raise ValueError("Fatal Error: Quaternion magnitude is zero. Simulation halted.")
    else:
        normalised_q = quaternion / magnitude
        new_state[6:10] = normalised_q

        return new_state, fuel_mass_remaining, new_state[13], altitude
