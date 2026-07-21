import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

#the cvs file must statisfy following conditions
# a header row that will be ingnored
# the columns must be arranged in the following order: mach number, cd values, cn values
 
#aero_table = np.loadtxt('gemini_saturn_v_coeff.csv', delimiter=',', skiprows=1)

# Now aero_table is a pure 2D math matrix
#mach_numbers = aero_table[:, 0]  
#cd_values = aero_table[:, 1]     
#cn_values = aero_table[:, 2]     

def binary_search_array(value, array):
    # returns index of value, or two indicies the value lies within, in an array
    length = len(array)
    
    # consdier edge cases first, set to boundry coeff values for simplicity (for now)

    if value <= array[0]:
        return 0, 0
    
    elif value >= array[-1]:
        return length - 1, length - 1 # max index = length of array -1
    
    # normal case

    else:

        left = 0
        right = len(array) - 1

        while left <= right:
            mid = (left + right) // 2 # Integer division
        
            if array[mid] == value:
                return mid, mid
            elif array[mid] < value:
                left = mid + 1
            else:
                right = mid - 1
            
        # When the loop breaks, 'right' is the lower bound, 'left' is the upper bound
        return right, left
         
\

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