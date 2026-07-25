import numpy as np

class PID_controller:

    """ Class containing PID contoller attributes """

    def __init__(self, kp, ki, kd, target, max, min): # target is the target vector orientation, must be a list

        self.kp = kp 
        self.ki = ki 
        self.kd = kd 
        self.target = np.array(target)
        self.integral_history = 0.0 # define integral history for integral controller
        self.prev_error = 0.0       # hold previous error to use in derivative controller
        # output limits, max and min values the output can have
        self.out_min = min
        self.out_max = max


    def correction(self, current_value, dt):

        """ Function returning the correcion required to get the current state to the target """

        current_error = self.target - current_value

        p_value = self.kp * current_error # proportional contribution

        # integral contribution, based of accumulation of previous values
        temporary_integral = self.integral_history + current_error * dt
        i_value = self.ki * temporary_integral

        # derivative contribution, predicting future error based off current gradient
        current_grad = ( current_error - self.prev_error ) / dt
        d_value = self.kd * current_grad

        self.prev_error = current_error # save after use

        raw_correction = p_value + i_value + d_value

        # --- ANTI-WINDUP AND CLAMPING LOGIC ---
        if raw_correction > self.out_max:
            # the engine is maxed out in the positive direction
            total_correction = self.out_max
            # DO NOT save the temp_integral, stop winding up
            
        elif raw_correction < self.out_min:
            # the engine is maxed out in the negative direction
            total_correction = self.out_min
            # DO NOT save the temp_integral, stop winding up
            
        else:
            # the engine is operating within its normal mechanical limits
            total_correction = raw_correction
            # safe to save the integral history
            self.integral_history = temporary_integral

        return total_correction