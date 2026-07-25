import numpy as np
from Coeff_fitting import binary_search_array as bsa
from Coeff_fitting import lerp

\

def get_CoG_fuel_bf(tank_height, rocket_bottom_to_tank, current_fuel_mass, max_fuel_mass):

    """ This function returns the CoG of the fuel only in the rocket body frame """

    # work out height of fuel in container currently, tank_height is the hight of the tank occupied by fuel at launch
    # assume that any buffers occupy negligible volume
    
    fuel_density = max_fuel_mass / tank_height                  # no CSA accounted for as will cancel out in the next line
    current_fuel_height = ( current_fuel_mass / fuel_density )
    CoG = rocket_bottom_to_tank + ( current_fuel_height / 2)    # assuming density is isotropic
    CoG_vec = np.array([float(CoG), 0, 0])                      # assuming sloshing is negligible, CoG lies along rockets long axis, assuming tank is axi-symmetric

    return CoG_vec # returns a numpy array, instead of a list, to work better with calculations

\

def get_CoG_total(dry_mass, CoG_dry_bf, fuel_mass, CoG_fuel_bf):
    
    """ This function returns the position vector of the overall centre of gravity of the rocket in the rocket BF relative to the bottom of the rocket """
    # note that vectors must be in a numpy arra, not a python list

    r_CoG = ( ( dry_mass * CoG_dry_bf ) + ( fuel_mass * CoG_fuel_bf) ) / ( dry_mass + fuel_mass )

    return r_CoG

\

def get_CoP(mach_num, aoa, mach_numbers, aoa_values, coeff_matrix, rocket_height):

    """ Function returning the current CoP of the rocket, based on mach number and angle of attack, in rocket body frame """
    
    # ensure input values never exceed the min/max limits of .csv tables
    mach_num = np.clip(mach_num, mach_numbers[0], mach_numbers[-1])
    aoa = np.clip(aoa, aoa_values[0], aoa_values[-1])

    mach_index_range = bsa(mach_num, mach_numbers)
    aoa_index_range = bsa(aoa, aoa_values)

    # extract base indices
    mach_idx1 = mach_index_range[0]
    aoa_idx1 = aoa_index_range[0]

    # 2. --- INDEX CLAMPING ---
    # read the exact dimensions directly from the matrix shape (Rows = AoA, Cols = Mach)
    num_aoa_rows, num_mach_cols = coeff_matrix.shape

    # force base indices to stop at least 1 step before the matrix edge
    aoa_idx1 = min(max(0, aoa_idx1), num_aoa_rows - 2)
    mach_idx1 = min(max(0, mach_idx1), num_mach_cols - 2)

    # define safe upper bounds for interpolation
    aoa_idx2 = aoa_idx1 + 1
    mach_idx2 = mach_idx1 + 1

    # extract physical values for the denominators 
    m1, m2 = mach_numbers[mach_idx1], mach_numbers[mach_idx2]
    a1, a2 = aoa_values[aoa_idx1], aoa_values[aoa_idx2]

    # nearest 4 coefficients
    C_11 = coeff_matrix[aoa_idx1][mach_idx1] # Bottom-Left
    C_21 = coeff_matrix[aoa_idx2][mach_idx1] # Top-Left
    C_12 = coeff_matrix[aoa_idx1][mach_idx2] # Bottom-Right
    C_22 = coeff_matrix[aoa_idx2][mach_idx2] # Top-Right

    # apply linear interpolation across AoA at the lower Mach boundary
    R1 = lerp(aoa, a1, a2, C_11, C_21)

    # apply linear interpolation across AoA at the upper Mach boundary
    R2 = lerp(aoa, a1, a2, C_12, C_22)

    # apply final interpolation to new R points across the Mach number
    R3 = lerp(mach_num, m1, m2, R1, R2)

    # table calculates CoP from the tip of the rocket
    CoP = rocket_height - R3
    CoP_vector = np.zeros(3)
    CoP_vector[0] = CoP

    return CoP_vector