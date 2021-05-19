#!/usr/bin/env python
# coding: utf-8

# In[20]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
# #     temp = np.ones(shape)
#     y = shape[0]
#     x = shape[1]
#     sigx = x*100
#     sigy = y//3
#     xx,yy = np.meshgrid(range(x), range(y))
#     steer_matrix_left_lane = -np.exp(- ((xx-x/2) / sigx)**2   - ((yy-y) / sigy) **2) 
#     print(steer_matrix_left_lane.shape)
#     plt.imshow(steer_matrix_left_lane)
#     print(steer_matrix_left_lane.shape)
#     print(temp.shape)
    factor = 0.2
    steer_matrix_left_lane = -factor * np.ones(shape)
    return steer_matrix_left_lane

# In[21]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
#     y = shape[0]
#     x = shape[1]
#     sigx = x*100
#     sigy = y//3
#     yy,xx = np.meshgrid(range(x), range(y))
#     steer_matrix_right_lane = np.exp(- ((xx-x/2) / sigx)**2   - ((yy-y) / sigy) **2)
#     plt.imshow(steer_matrix_right_lane)
    factor = 0.2
    steer_matrix_right_lane = factor*np.ones(shape)
    return steer_matrix_right_lane

# In[190]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape

    # Convert to HsV 
    
    imagehsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    # Gaussian 
    sigma = 2.5
    img_gaussian_filter = cv2.GaussianBlur(img, (0,0), sigma)
    
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    mask_mag = Gmag > [[30]]
    
    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    # Left / Right Mask
#     print(sobelx.shape)
    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(5*width//8)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(3*width//8))] = 0
    # Edge Mask 
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    # Color Mask 
#     white_lower_hsv = np.array([2, 2, 100])         # CHANGE ME
    white_lower_hsv = np.array([0, 0, 100])         # CHANGE ME
    white_upper_hsv = np.array([160, 55, 250])   # CHANGE ME
    
#     yellow_lower_hsv = np.array([20, 75, 120])        # CHANGE ME
#     yellow_upper_hsv = np.array([30, 255, 220])  # CHANGE ME

    yellow_lower_hsv = np.array([20, 25, 60])        # CHANGE ME
    yellow_upper_hsv = np.array([30, 255, 220])  # CHANGE ME
       
    mask_white = cv2.inRange(imagehsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imagehsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = np.ones([h, w]) * mask_mag * mask_yellow * mask_left #* mask_sobelx_neg 
    mask_right_edge = np.ones([h, w]) * mask_mag * mask_white * mask_right #* mask_sobelx_pos 

#     mask_left_edge = np.ones([h, w]) 
#     mask_right_edge = np.ones([h, w])
    
    return (mask_left_edge, mask_right_edge)
