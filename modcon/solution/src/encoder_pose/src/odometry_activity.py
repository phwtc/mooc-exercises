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
    
# Solution     
#     # Read the number of ticks
#     N_tot = encoder_msg.resolution #total number of ticks per wheel revolution
#     ticks = encoder_msg.data

#     alpha = 2*np.pi/N_tot # rotation per tick in radians 
#     delta_ticks = ticks-prev_ticks    
#     delta_phi = alpha*delta_ticks # in radians
    
    
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
    
    x_curr = dA * np.cos(theta_prev) + x_prev
    y_curr = dA * np.sin(theta_prev) + y_prev
    theta_curr = theta + theta_prev
#     # r = 0 # make different than zero if you have reason to believe the wheels are of different sizes.
#     R_left = R # * (1-r)
#     R_right = R # * (1+r)
#     d_left = R_left * delta_phi_left 
#     d_right = R_right * delta_phi_right
    
#     # Define distance travelled by the robot, in body frame [m]
#     d_A = (d_left + d_right)/2
#     # Define rotation of the robot [rad]
    
#     Dtheta = (d_right - d_left)/baseline_wheel2wheel
    
#     # Define distance travelled by the robot, in world frame [m]
    
#     Dx = d_A * np.cos(theta_prev)
#     Dy = d_A * np.sin(theta_prev)
    
#     # Update pose estimate
    
#     x_curr = x_prev + Dx
#     y_curr = y_prev + Dy
#     theta_curr = theta_prev + Dtheta

    return x_curr, y_curr, theta_curr
