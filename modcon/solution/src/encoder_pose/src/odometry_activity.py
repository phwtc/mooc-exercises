#!/usr/bin/env python
# coding: utf-8

# In[13]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

#TODO: write a correct function

def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg: ROS encoder message (ENUM)
            prev_ticks: Previous tick count from the encoders (int)
        Return:
            rotation_wheel: Rotation of the wheel in radians (double)
            ticks: current number of ticks (int)
    """
    # TODO: these are random values, you have to implement your own solution in here
    N_tot = encoder_msg.resolution # number of ticks per wheel revolution
    ticks = encoder_msg.data # incremental count of ticks from the encoder

    
    # Convert ticks to rotation 
#     N_tot = 135 # total number of ticks per revolution
    alpha = np.pi * 2 / N_tot # wheel rotation per tick in radians
    
    delta_phi = (ticks - prev_ticks) * alpha
#     ticks = ticks - prev_ticks
    # print(prev_ticks, ticks, delta_phi)
    
    
    
    
    
    return delta_phi, ticks

# In[11]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your odometry function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write the odometry function

import numpy as np 

def poseEstimation( R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
                    baseline_wheel2wheel, # distance from wheel to wheel; 2L of the theory
                    x_prev, # previous x estimate - assume given
                    y_prev, # previous y estimate - assume given
                    theta_prev, # previous orientation estimate - assume given
                    delta_phi_left, # left wheel rotation (rad)
                    delta_phi_right): # right wheel rotation (rad)
    
    """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
#     R*rotation_wheel_left / (2*np.pi)
    
    dl = delta_phi_left * R 
    dr = delta_phi_right * R 
    theta = (dr - dl) / (baseline_wheel2wheel)
    dA = (dl+dr) / 2
    
#     print(dl, dr)
    theta_curr = theta + theta_prev
    x_curr = dA * np.cos(theta_curr) + x_prev
    y_curr = dA * np.sin(theta_curr) + y_prev
    
#     print(x_curr, y_curr)

    return x_curr, y_curr, theta_curr
