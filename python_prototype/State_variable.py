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

def get_coeff_value(mach_num, array_mn, array_coeff):
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

m_rocket_dry = 10e4
m_max_fuel = m_rocket_dry / 0.95

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
    current_isp = ISP_VAC + (ISP_SL - ISP_VAC) * (pressure / P_ATM)
    
    return current_isp

def get_current_density(altitude):

    density, _, _ = get_atmosphere(altitude) # in kg/ cu m

    return density


# ---------------------------------------------------------
# 1. DEFINE THE INITIAL STATE VECTOR (14 Variables)
# ---------------------------------------------------------

# [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz, mass], ECI refers to earth centred inertial frame
initial_state = np.array([
    6378137.0, 0.0, 0.0,   # 0:2 - Position (ECI meters)
    0.0, 0.0, 0.0,         # 3:5 - Velocity (ECI m/s)
    1.0, 0.0, 0.0, 0.0,    # 6:9 - Quaternion (Orientation)
    0.0, 0.0, 0.0,         # 10:12 - Angular Velocity (Body rad/s)
    500000.0               # 13 - Mass (kg)
])


# ---------------------------------------------------------
# 2. THE CORE DERIVATIVE FUNCTION
# ---------------------------------------------------------
def get_derivatives(t, state, mfr):
    """ Takes the current state array and returns the rates of change. """

    # unpack the state vector for readability
    position = state[0:3]
    velocity = state[3:6]
    q = state[6:10]      # q[0] is scalar, q[1:4] are vector
    omega = state[10:13] # angular velocity
    m = state[13]

    # wind velocity = 0 for now

    v_wind = np.zeros(3)

    # initialize the array that will hold our output derivatives
    state_dot = np.zeros(14)
    
    # --- TRANSLATIONAL KINEMATICS ---
    # the derivative of position is simply the current velocity
    state_dot[0:3] = velocity

    altitude = get_true_altitude(position)
    isp = get_current_isp(altitude)
    
    # thrust vector in ECI, no gimbaling of engine currently!
    thrust_mag = mfr * isp * G0
    thrust_direction_eci = np.array([
        1 - 2*(q[2]**2 + q[3]**2),
        2*(q[1]*q[2] + q[0]*q[3]),
        2*(q[1]*q[3] - q[0]*q[2])
    ])
    thrust_force_eci = thrust_mag * thrust_direction_eci
    
    # gravitational force in ECI
    grav_force_eci = grav_acc(position, t) * m # inherently has a negative direction
    
    # aerodynamic forces

    altitude_const = .5 * get_current_density(altitude) * CSA 

    v_rel = velocity - v_wind
    v_rel_mag = np.linalg.norm(v_rel)
    _, _, temperature =  get_atmosphere(altitude)
    temperature += 273.15 # convert temperature from celcius to kelvin
    v_sound = np.sqrt( 1.4 * 287.05 * temperature ) # speed of sound varies with altitude
    v_rel_mag_mach = v_rel_mag * v_sound # coverting to mach number

    drag_force_eci = -altitude_const * v_rel_mag * v_rel * get_coeff_value(v_rel_mag_mach, mach_numbers, cd_values) 
    #lift_force = 


    # Net Force and Acceleration
    net_force_eci = thrust_force_eci + grav_force_eci + drag_force_eci
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

    # check amount of fuel remaining

    fuel_mass_remaining = state[13] - dry_mass
    if fuel_mass_remaining < mfr:
        mfr_adjusted = fuel_mass_remaining
    else:
        mfr_adjusted = mfr

    # main RK4

    k1 = get_derivatives(t, state, mfr_adjusted)
    k2 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k1) / 2) ), mfr_adjusted )
    k3 = get_derivatives( ( t + dt / 2 ) , ( state + ( (dt * k2) / 2) ), mfr_adjusted )
    k4 = get_derivatives( t + dt , ( state +  (dt * k3) ), mfr_adjusted )

    new_state = state + ( ( dt / 6 ) * ( k1 + ( 2 * k2 ) + ( 2 * k3 ) + k4 ) )

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
