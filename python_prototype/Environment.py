import math
import matplotlib.pyplot as plt
import numpy as np

# Constants

# Precomputed Earth Standard Gravitational Parameter (m^3/s^2)
MU = 3.986004418e14 

m_rocket = 10e4
m_max_fuel = m_rocket / 0.95

# WGS84 Earth reference ellipsoid constants (in meters)
A_EQ = 6378137.0         # Equatorial radius
B_POL = 6356752.314245   # Polar radius

def mass(time):
    # use a linear decrease for now just for simplicity
    gradient = 10000 # 1000kg/s of fuel burned
    total_mass =  m_rocket + ( m_max_fuel - time * gradient ) 
    if total_mass >= m_rocket:
        return total_mass
    else:
        return m_rocket
    
\

def get_earth_radius(position):
    """ Calculates the exact radius of the Earth in the direction of the given ECI position vector. """
    # 1. Calculate the distance from the center of the Earth
    dist = np.linalg.norm(position)
    
    # Safety check: prevent division by zero if at the exact center of the Earth
    if dist == 0:
        return 0.0
        
    # 2. Normalize the position vector to get the pure direction (ux, uy, uz)
    u = position / dist
    
    # 3. Calculate the radius using the oblate spheroid equation
    term1 = (u[0]**2 + u[1]**2) / (A_EQ**2)
    term2 = (u[2]**2) / (B_POL**2)
    
    radius = 1.0 / np.sqrt(term1 + term2)
    
    return radius

\

def get_true_altitude(position):
    """
    Calculates the true altitude of the rocket above the oblate Earth surface.
    """
    dist_from_center = np.linalg.norm(position)
    local_earth_radius = get_earth_radius(position)
    
    return dist_from_center - local_earth_radius

\

def grav_acc(position, t):
    # position should be a numpy array: np.array([x, y, z])
    distance = np.linalg.norm(position)
    
    # Normalized vector pointing toward the origin (Earth's center)
    direction_vec_grav = -position / distance
    
    # Force of gravity vector
    return (MU / (distance**2)) * direction_vec_grav

\

def get_atmosphere(alt): # may be more useful to change this to a function of just position insteaad of altitude later on

    # VACUUM CUTOFF: Above 85,000 meters, air density is negligible.
    if alt > 85000:
        return 0.0, 0.0, -273.15  # Return density, pressure, and absolute zero temp
        
    # uses https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    # temperature in deg C, pressure in KPa
    if alt <= 11000:
        temperature = 15.04 - (0.00649 * alt)
        pressure = 101.29 * (((temperature + 273.1) / 288.08) ** 5.256)
        
    elif alt > 11000 and alt <= 25000:
        temperature = -56.46
        pressure = 22.65 * np.exp(1.73 - 0.000157 * alt)
        
    else:
        temperature = -131.21 + (0.00299 * alt)
        pressure = 2.488 * (((temperature + 273.1) / 216.6) ** -11.388)

    # Calculate density (kg/m^3)
    density = pressure / (0.2869 * (temperature + 273.1))
    
    # Return all three atmospheric properties 
    return density, pressure, temperature
