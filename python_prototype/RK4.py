import math
import numpy as np
from State_variable import get_derivatives

def RK4_new_state(t, state, dt, mfr, dry_mass): 

    # check amount of fuel remaining

    fuel_mass_remaining = state[13] - dry_mass

    if fuel_mass_remaining < mfr:
        mfr_adjusted = fuel_mass_remaining

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
