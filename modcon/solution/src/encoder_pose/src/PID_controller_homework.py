#!/usr/bin/env python
# coding: utf-8

# In[24]:


### import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
#     (0.2,10, 15, 0.3)
#     (0.2, 15,20, 0.5)
#     (0.5, 15,30, 0.5)
# -0.005, 0.5, 10, 1 
# -0.1, 2, 8, 1, works oK 
     # TODO: these are random values, you have to implement your own PID controller in here
    #ku = 1
    #tu = 15
    
    #k_i = 1.2*ku/tu
    #k_p = ku *0.6
    #k_d = 3*ku*tu/40
#     kp =3 period = 220s

    # Good
    k_i = 0.2
    k_p = 1.8
    k_d = 183
    over = 1

    # Good
    k_i = 0.2
    k_p = 1.5
    k_d = 200
    over = 1

#     k_i = 0.2
#     k_p = 1.35
#     k_d = 166
#     over = 1
    
    
#     print(y_ref, y_hat)
    e = y_ref-y_hat
    e_int = prev_int_y + e * delta_t
    e_der = (e - prev_e_y) / delta_t 
    e_int = max(min(e_int,2),-2)
    
#     if abs(e) < 0.03:
#         over = -over
    omega = over*(k_p* e + k_i * e_int + k_d * e_der)
#     print(f'omega: {omega}')
    
#     omega = max(min(omega,8), -8)
#     if omega > 8: 
#         print(omega)
    
    return [v_0, omega], e, e_int

