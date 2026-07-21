import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

from Environment import get_atmosphere
from Environment import mass
from Environment import get_earth_radius
from Environment import get_true_altitude
from Environment import grav_acc

from Coeff_fitting import binary_search_array

# cvs file for cd and cn data goes here:

# the cvs file must statisfy following conditions
# a header row that will be ingnored
# the columns must be arranged in the following order: mach number, cd values, cn values
 
# aero_table = np.loadtxt("C:\Codes\Rocket_project\coeff_cvs\gemini_saturn_v_coeff.csv", delimiter=',', skiprows=1)
df = pd.read_csv(r"C:\Codes\Rocket_project\coeff_cvs\gemini_saturn_v_coeff.csv")
aero_table = df.to_numpy()

mach_numbers = aero_table[:, 0]  
cd_values = aero_table[:, 1]     
cn_values = aero_table[:, 2]     

\

def dot_prod(vector1, vector2):
    """ Allows for dot product between 2 vectors """
    product = 0

    if len(vector1) == len(vector2):

        for i in range(0, len(vector1)):

            pair_product = vector1[i] * vector2[i]
            product += pair_product

        return float(product)

    else:
        return "Cannot take dot product of vectors of different dimensions"

\

def get_coeff_value(mach_num, array_mn, array_coeff):
    # !!!IMPORTANT!!! Note that the C_n values are actually given as a gradient per unit radian
    # note that currently doesn't account for angle of attack
    # uses linear interpolation if speed is not in table
    index_range = binary_search_array(mach_num, array_mn)

    if index_range[0] == index_range[1]:
        coeff = array_coeff[index_range[1]]
    
    else:
        grad = ( array_coeff[index_range[1]] - array_coeff[index_range[0]] ) / ( array_mn[index_range[1]] - array_mn[index_range[0]] )
        coeff = array_coeff[index_range[0]] + ( grad * (mach_num - array_mn[index_range[0]]) )
        
    return coeff

# Constants

# Precomputed Earth Standard Gravitational Parameter (m^3/s^2)
MU = 3.986004418e14 

# WGS84 Earth reference ellipsoid constants (in meters)
A_EQ = 6378137.0         # Equatorial radius
B_POL = 6356752.314245   # Polar radius

# Constants for the SpaceX Merlin 1D Engine (example)
ISP_SL = 282.0   # seconds
ISP_VAC = 311.0  # seconds
P_ATM = 101.325 # Kilo-Pascals (standard sea level pressure)
G0 = 9.80665 # gravitational acceleration on earth's surface
CSA = 10.06 **2 * np.pi * .25 # based off Saturn V reference diameter

def get_current_isp(altitude):

    _, pressure, _ = get_atmosphere(altitude) # in kPa
    
    # Interpolate I_sp based on the pressure ratio assuming linear model
    current_isp = ISP_VAC + (ISP_SL - ISP_VAC) * (pressure / P_ATM) # ISP_SL stands for Specific Impulse at Sea Level
    
    return current_isp

def get_current_density(altitude):

    density, _, _ = get_atmosphere(altitude) # in kg/ cu m

    return density


# ---------------------------------------------------------
# THE CORE DERIVATIVE FUNCTION
# ---------------------------------------------------------

def get_derivatives(t, state, mfr):
    """ Takes the current state array and returns the rates of change. """

    # unpack the state vector for readability
    position = state[0:3]
    velocity = state[3:6]
    q = state[6:10]      # q[0] is scalar, q[1:4] are vector
    omega = state[10:13] # angular velocity
    m = state[13]

    # debugging mass
    #print(m)

    # wind velocity = 0 for now

    v_wind = np.zeros(3)

    # initialize the array that will hold our output derivatives
    state_dot = np.zeros(14)
    
    # --- TRANSLATIONAL KINEMATICS ---
    # the derivative of position is simply the current velocity
    state_dot[0:3] = velocity

    altitude = get_true_altitude(position)
    isp = get_current_isp(altitude)
    
    rocket_long_axis_direc_unnorm = np.array([ # long axis direction of rocket converted to the ecif 
        1 - 2*(q[2]**2 + q[3]**2),
        2*(q[1]*q[2] + q[0]*q[3]),
        2*(q[1]*q[3] - q[0]*q[2])
    ])

    rocket_long_axis_direc_eci = rocket_long_axis_direc_unnorm / np.linalg.norm(rocket_long_axis_direc_unnorm)

    # thrust vector in ECI, no gimbaling of engine currently!
    thrust_mag = mfr * isp * G0
    thrust_force_eci = thrust_mag * rocket_long_axis_direc_eci
    
    # gravitational force in ECI
    grav_force_eci = grav_acc(position, t) * m # inherently has a negative direction
    
    # aerodynamic forces

    altitude_const = .5 * get_current_density(altitude) * CSA 

    v_rel = velocity - v_wind
    v_rel_mag = np.linalg.norm(v_rel)

    if v_rel_mag > 0: # avoid any div-by-zero errors (specifically at start)

        v_rel_direc = v_rel / v_rel_mag
        _, _, temperature =  get_atmosphere(altitude)
        temperature += 273.15 # convert temperature from celcius to kelvin

        if temperature < 0.1:
            v_rel_mag_mach = 0 # this avoides the div by zero error for high altitudes causing temperature --> 0
        else:
            v_sound = np.sqrt( 1.4 * 287.05 * temperature ) # speed of sound varies with altitude
            v_rel_mag_mach = v_rel_mag / v_sound # coverting to mach number

        drag_force_eci = -altitude_const * v_rel_mag * v_rel * get_coeff_value(v_rel_mag_mach, mach_numbers, cd_values) 

        cos_aoa = np.clip( dot_prod(rocket_long_axis_direc_eci, v_rel_direc), -1.0, 1.0 ) # np.clip used so that no floating point errors from python cause value to have a mag > 1.0
        aoa = np.arccos(cos_aoa) # note np.arccos give radians
        lift_force_mag = altitude_const * (v_rel_mag ** 2) * ( get_coeff_value(v_rel_mag_mach, mach_numbers, cn_values) * aoa) # multiply by aoa as cn is given as a gradient per radian
        lift_force_resolved = rocket_long_axis_direc_eci - ( cos_aoa * v_rel_direc) # this is the vector that points in the direction of the lift force, but it is NOT normalised
        lift_force_resolved_mag = np.linalg.norm(lift_force_resolved)

        if lift_force_resolved_mag < 1e-8: # if rocket orientated directly up (relative to earth), lift force is zero, have small value to avoid float point errors
            lift_force_eci = np.zeros(3)

        else:
            lift_force_resolved_norm = lift_force_resolved / lift_force_resolved_mag
            lift_force_eci = lift_force_mag * lift_force_resolved_norm

    else:
        lift_force_eci = np.zeros(3)
        drag_force_eci = np.zeros(3)


    # Net Force and Acceleration
    net_force_eci = thrust_force_eci + grav_force_eci + drag_force_eci + lift_force_eci
    acceleration_eci = net_force_eci / m
    
    # The derivative of velocity is acceleration
    state_dot[3:6] = acceleration_eci
    
    # --- ROTATIONAL KINEMATICS (Placeholder for now) ---
    # We will implement quaternion derivative math here later
    state_dot[6:10] = np.zeros(4) 
    
    # --- MASS KINEMATICS ---
    # The derivative of mass is the negative mass flow rate
    state_dot[13] = -mfr 
    
    return state_dot

# ---------------------------------------------------------
# 3. The RK4 integrator
# ---------------------------------------------------------

def RK4_new_state(t, state, dt, mfr, dry_mass): 

    # check how much fuel is left (in kg)
    fuel_mass_remaining = state[13] - dry_mass

    print(f" ------------------------------------------------------------------------------ \n Time = {t}, Dry mass = {dry_mass}, Fuel mass = {fuel_mass_remaining}, Total mass = {state[13]}") # mission control display

    # check how much fuel the engine can burn this frame (in kg) based off mfr
    fuel_needed_this_step = mfr * dt

    # calculate adjusted mass flow rate
    if fuel_mass_remaining <= 0.0:
        # Tank is empty. Shut the engine down.
        mfr_adjusted = 0.0
        
    elif fuel_mass_remaining < fuel_needed_this_step:
        # Not enough fuel for a full time step. 
        # Set mfr so we burn EXACTLY what is left over the duration of 'dt'
        mfr_adjusted = fuel_mass_remaining / dt
        
    else:
        # Plenty of fuel. Run at normal throttle.
        mfr_adjusted = mfr

    # main RK4

    #print(f"Before RK4 calc: Dry mass = {dry_mass}, state mass = {state[13]}")
    #print(f"The mfr is: {mfr_adjusted} and the initial was {mfr}") # debugging code
    k1 = get_derivatives(t, state, mfr_adjusted)
    k2 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k1) / 2) ), mfr_adjusted )
    k3 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k2) / 2) ), mfr_adjusted )
    k4 = get_derivatives( t + dt , ( state +  (dt * k3) ), mfr_adjusted )

    new_state = state + ( ( dt / 6 ) * ( k1 + ( 2 * k2 ) + ( 2 * k3 ) + k4 ) )
    #print(f"After RK4 calc: Dry mass = {dry_mass}, state mass = {new_state[13]}")

    # we must normalise the quaternion magnitude so that it represents a true rotation only

    quaternion = new_state[6:10]
    magnitude = np.linalg.norm(quaternion)

    # selection to notify of any div by zero errors, clearly code is buggy if mag = 0
    if magnitude == 0:
        raise ValueError("Fatal Error: Quaternion magnitude is zero. Simulation halted.")
    else:
        normalised_q = quaternion / magnitude
        new_state[6:10] = normalised_q

        return new_state
