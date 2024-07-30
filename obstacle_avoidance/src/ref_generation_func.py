import math
import numpy as np
import change_of_coordinates_func as cc
from parameters import *

def ref_generation(path, N, current_x, current_y, current_yaw, error_yaw_max, surge_max, surge_min, MODE_YAW, MODE_SURGE, MODE_ZENO, ANGLE_OUTPUT):

    #------------------------------------------------------------------------------
    # COMPUTING THE YAW ANGLE RELATIVE ERROR REFERENCE    
    #------------------------------------------------------------------------------ 
    # "N" is the number of the selected "closest" (from the current position of Zeno) waypoints. 

    closest_WP_list = []    # Inizialization of a list of the selected closest way_points
    if len(path) <= N + 1:  # If we are too close to the obstacle, we can't extract the next "N" waypoints..
        N = 1
    for i in range(0, N):
        cell_WP = path[i]            # "cell_WP" because it contains the (row,col) of the cell of the grid containg that specific way-point
        [x_WP, y_WP] = cc.grid_to_local(cell_WP[0], cell_WP[1], res_meters)     # conversion to local coordinates of each WP instead of row and col
        closest_WP_list.append([x_WP, y_WP])                                    
    
    #------------------------------------------------------------------------------
    # Loop for weighting each way-point (I assumed different ways to give weights)
    yaw_ref = 0.0               # yaw_ref initialization
    w = N                       # weight initialization for decreasing method
    for WP in closest_WP_list:
        angle = math.atan2(WP[1] - current_y, WP[0] - current_x)    # angle in rad from current position to the selected WP in the list of WPs

        if MODE_YAW == "uniform":
            weight = 1.0/N                  # uniform weight for the closest Way Points
            yaw_ref = np.arctan2(np.sin(yaw_ref + angle * weight), np.cos(yaw_ref + angle * weight))      # incrementing the sum of weighted angles
        elif MODE_YAW == "decreasing":          # This mode gives more weights to the closest WPs
            yaw_ref = np.arctan2(np.sin(yaw_ref + w * angle / (N * (N + 1)) * 2), np.cos(yaw_ref + w * angle / (N * (N + 1)) * 2))
            #yaw_ref += w * angle / (N * (N + 1)) * 2    # (N*(N+1))/2 is the formula for the sum of the first N integers
            w = w - 1                       # Decreasing the weight at each step
        else:
            print('WRONG STRING FOR MODE_YAW, SET IT TO "uniform" or "decreasing"')
            return 0, 0   
    err_yaw = np.arctan2(np.sin(yaw_ref - current_yaw), np.cos(yaw_ref- current_yaw))  
    # saturation limits (set to 30 deg at the moment in the code) 
    if err_yaw < -error_yaw_max:
        err_yaw = -error_yaw_max
    elif err_yaw > error_yaw_max:
        err_yaw = error_yaw_max

    
    #------------------------------------------------------------------------------
    # COMPUTING THE SURGE SPEED REFERENCE    
    #------------------------------------------------------------------------------
    surge = 0.0         # surge speed initialization

    if MODE_SURGE == "steer sensitive speed":       # This control law slow down Zeno to a minimum if he tries to steer fast
        abs_ref_err_yaw = np.abs(err_yaw)
        if abs_ref_err_yaw >= error_yaw_max:
            surge = surge_min
        else:
            surge = surge_max - abs_ref_err_yaw*(surge_max - surge_min)/error_yaw_max
            if surge <= surge_min:
                surge = surge_min
    elif MODE_SURGE == "constant":       
        surge = 0.1
    else:
            print('WRONG STRING FOR MODE_SURGE, SET IT TO "steer sensitive speed"')
            return 0, 0
    if MODE_ZENO == "stop":        # ZENO SAFETY BUTTON :)
        err_yaw = 0.0
        surge = 0.0

    if ANGLE_OUTPUT == "deg":
        err_yaw = err_yaw*180/math.pi

    return err_yaw, surge
    

 