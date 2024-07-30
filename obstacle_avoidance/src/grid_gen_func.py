

import numpy as np
import math

# Custom Imports
import change_of_coordinates_func as cc



#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# (FOR TESTING PURPOSES) GRID GENERATING FUNCTIONS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def place_obstacle_square(grid, lat_obs, lon_obs, lat_min, lon_min, res_dec_deg, res_meters, dim_obs):
    """
    
    This function simulates the presence of a squared shape obstacle and sets as 1
    the corresponding cells in the grid.

    Arguments
        - grid: This is the grid representing the map. It must be passed 
                as a numpy ndarray with dtype=np.int8 (8-bit integers). 
                The elements of the grid must be 0 or 1, indicating 
                respectively free cell, or occupied cell.
        - lat_obs: latitude of the centre of the obstacle, expressed in [_ deg]
        - lon_obs: longitude of the centre of the obstacle, expressed in [_ deg]
        - lat_min: latitude lower limit of map, expressed in [_ deg]
        - lon_min: longitude lower limit of map, expressed in [_ deg]
        - res_dec_deg: resolution of a cell, expressed in [_ deg]
        - res_meters: dimension of one grid cell, expressed in [m]
        - dim_obs: length of the side of the square, expressed in [m]
    
    Outputs
        - no outputs

    """
    
    # Compute discrete coordinates of obstacle centre
    [row_obs, col_obs] = cc.nav_to_grid(lat_obs, lon_obs, lat_min, lon_min, res_dec_deg)

    # Compute discrete lengths of side of square
    dim_obs_grid = np.ceil(dim_obs/res_meters)

    # Set proper cells of grid to 1 (the one belonging to square)
    for i in np.arange(-np.floor(dim_obs_grid/2), np.floor(dim_obs_grid/2)+1):
        for j in np.arange(-np.floor(dim_obs_grid/2), np.floor(dim_obs_grid/2)+1):
            grid[int(row_obs+i), int(col_obs+j)] = 1


def place_obstacle_circle(grid, lat_obs, lon_obs, lat_min, lon_min, res_dec_deg, res_meters, dim_obs):
    """
    
    This function simulates the presence of a circle shape obstacle and sets as 1
    the corresponding cells in the grid.

    Arguments
     - grid: This is the grid representing the map. It must be passed 
            as a numpy ndarray with dtype=np.int8 (8-bit integers). 
            The elements of the grid must be 0 or 1, indicating 
            respectively free cell, or occupied cell.
     - lat_obs: latitude of the centre of the obstacle, expressed in [_ deg]
     - lon_obs: longitude of the centre of the obstacle, expressed in [_ deg]
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - res_dec_deg: resolution of a cell, expressed in [_ deg]
     - res_meters: dimension of one grid cell, expressed in [m]
     - dim_obs: radius of the circle, expressed in [m]
    
    Outputs
     - no outputs

    """
    
    # Compute discrete coordinates of obstacle centre
    [row_obs, col_obs] = cc.nav_to_grid(lat_obs, lon_obs, lat_min, lon_min, res_dec_deg)

    # Compute discrete length of radius of circle
    dim_obs_grid = np.ceil(dim_obs/res_meters)

    # Set proper cells of grid to 1 (the one belonging to circle)
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            # Calculate the distance from the center of the ellipse to the current cell
            distance = (i - row_obs)**2 + (j - col_obs)**2
            # If the distance is less than or equal to 1, set the cell to 1
            if distance <= dim_obs_grid:
                grid[i, j] = 1


def create_random_grid(lat_min, lat_max, lon_min, lon_max, res_dec_deg, res_meters, r_earth, n_obs):
    """
    
    This function creates and returns a grid representing the map with some random 
    placed squared and circle obstacles

    Arguments:
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lat_max: latitude upper limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - lon_max: longitude upper limit of map, expressed in [_ deg]
     - res_dec_deg: resolution of a cell, expressed in [_ deg]
     - res_meters: dimension of one grid cell, expressed in [m]
     - r_earth: radius of earth at (43_ N) latitude, expressed in [m]
     - n_obs: number of square and circle obstacles to place

    Outputs:
     - grid: This is the grid representing the map. It must be passed 
             as a numpy ndarray with dtype=np.int8 (8-bit integers). 
             The elements of the grid must be 0 or 1, indicating 
             respectively free cell, or occupied cell.
    
    """
    n_rows = int(np.ceil((lat_max - lat_min)/res_dec_deg))
    n_cols = int(np.ceil((lon_max - lon_min)/res_dec_deg))

    lat_range = lat_max - lat_min
    lon_range = lon_max - lon_min

    x_range = lat_range*math.pi*r_earth/180

    grid = np.zeros((n_rows, n_cols), dtype=np.int8)

    for i in range(n_obs):
        lat_obs = lat_min + np.random.uniform(0.25, 0.80)*lat_range
        lon_obs = lon_min + np.random.uniform(0.25, 0.80)*lon_range
        dim_obs = np.random.uniform(0.01, 0.15)*x_range
        place_obstacle_square(grid, lat_obs, lon_obs, lat_min, lon_min, res_dec_deg, res_meters, dim_obs)

    for j in range(n_obs):
        lat_obs = lat_min + np.random.uniform(0.25, 0.80)*lat_range
        lon_obs = lon_min + np.random.uniform(0.25, 0.80)*lon_range
        dim_obs = np.random.uniform(0.01, 0.15)*x_range
        place_obstacle_circle(grid, lat_obs, lon_obs, lat_min, lon_min, res_dec_deg, res_meters, dim_obs)       

    return grid


