def lerp(x, x0, x1, y0, y1):
    """ Function for 1D Linear Interpolation """
    # Prevent Division by Zero if the target falls exactly on a grid line
    if x0 == x1: 
        return y0
    
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

\

def binary_search_array(value, array):

    """ Returns index of value, or two indicies the value lies within, in an array """

    length = len(array)
    
    # consdier edge cases first (set to boundry coeff values for simplicity)

    if value <= array[0]:
        return 0, 0
    
    elif value >= array[-1]:
        return length - 1, length - 1 # max index = length of array -1
    
    # normal case

    else:

        left = 0
        right = len(array) - 1

        while left <= right:
            mid = (left + right) // 2 
        
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

    """ Function retuning the approximate value of a cn/ cd coefficient based off current mach number """

    # uses linear interpolation if speed is not in table
    index_range = binary_search_array(mach_num, array_mn)

    # extract the boundary indices
    idx0 = index_range[0]
    idx1 = index_range[1]

    # extract the exact physical X and Y values
    x0 = array_mn[idx0]
    x1 = array_mn[idx1]
    y0 = array_coeff[idx0]
    y1 = array_coeff[idx1]

    # use lerp function to calculate and return the interpolated value
    coeff = lerp(mach_num, x0, x1, y0, y1)
    
    return coeff