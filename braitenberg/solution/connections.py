from typing import Tuple
# from preprocessing import preprocess
import numpy as np


N = 40
M = 40
bound = 2
centers = [ (N //2, 6*M//8), (N//8, 5*M//8) , (7*N//8, 5*M//8)]
spread = [(3,7), (4,5), (4,5)]
def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # Implement chaser 
    # Idea 
    # Try to find where is the target and put it to center 
    # If target is on left side, turn left --> 
    # small value on left motor , large value on right motor
    
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    rows = shape[0] // M 
    cols = shape[1] // N

    signs = [1, 1, 0]
    
    # Steer away 
    for rr in range(M):
        for cc in range(N):
            for ind, center in enumerate(centers):
                x = (cc - center[0]) / spread[ind][0]
                y = (rr - center[1]) / spread[ind][1]
                res[rr*rows: (rr+1)*rows:, cc*cols:(cc+1)*cols] += signs[ind] * np.exp(-x*x - y*y)
        
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    rows = shape[0] // M 
    cols = shape[1] // N

    signs = [1, 0, 1]
    
    # Steer away 
    for rr in range(M):
        for cc in range(N):
            for ind, center in enumerate(centers):
                x = (cc - center[0]) / spread[ind][0]
                y = (rr - center[1]) / spread[ind][1]
                res[rr*rows: (rr+1)*rows:, cc*cols:(cc+1)*cols] += signs[ind] * np.exp(-x*x - y*y)
        
    return res
