#!/usr/bin/env python


# -------------------------------------------------------------------------------------------------------
# IMPORT NECESSARY LIBRARIES
# -------------------------------------------------------------------------------------------------------

import rospy
import numpy as np
import math
import change_of_coordinates_func as CC     # Library for change of coordinates functions
import rviz_util as ru
from parameters import *

# -------------------------------------------------------------------------------------------------------
# IMPORT ROS MESSAGES
# -------------------------------------------------------------------------------------------------------

from obstacle_avoidance.msg import MySensor, UpdatedCells
from nav_msgs.msg import OccupancyGrid


# -------------------------------------------------------------------------------------------------------
# VARIABLE INITIALIZING
# -------------------------------------------------------------------------------------------------------

coord_vert = []     # coordinates of the vertices of the real map

lon_vert = []       # longitude of the vertices of the real map
lat_vert = []       # latitude of the vertices of the real map

min_lon = []        # minimum longitude among the vertices of the real map
max_lon = []        # maximum longitude among the vertices of the real map
min_lat = []        # minimum latitude among the vertices of the real map
max_lat = []        # maximum latitude among the vertices of the real map

map_width = []      # [m] width of the virtual map
map_height = []     # [m] height of the virtual map

n_rows = []         # number of rows of the virtual map
n_columns = []      # number of columns of the virtual map

virtual_map = []    # matrix representing the virtual map
scaled_map = []     # matrix of 1 and 0 (obstacle and free)

free_threshold = 1  # Value which a cell that has been seen as free is lowered by

max_value_cell = 5 * number_of_beams # Maximum value a cell can assume



# -------------------------------------------------------------------------------------------------------
# AUXILIARY FUNCTIONS
# -------------------------------------------------------------------------------------------------------

def read_map():
    # This function reads a .txt file that contains the corners of a polygonal shaped map
    # remember to change the file path and use the correct one!!
    # nome_file = "obstacle_avoidance_soft.txt"
    # nome_file = "obstacle_avoidance_hard.txt"
    nome_file = "map_file.txt"

    # Open the text file
    with open(nome_file, 'r') as file:
        lines = file.readlines()

    map_data = []

    for index, line in enumerate(lines):
        # Strip leading and trailing whitespace from the line
        line = line.strip()

        # Extract map data from the first line
        if index == 0:
            if line.startswith("map"):
                map_data = eval(line.split('=')[1].strip())
                continue  # Move to the next line
    
    return map_data


def initialize_map():
    global coord_vert, lon_vert, lat_vert, min_lon, max_lon, min_lat, max_lat, map_width, map_height, n_rows, n_columns, scaled_map
    # get the coordinates of the vertices from the .txt file

    coord_vert = read_map()   
    lon_vert = [row[0] for row in coord_vert]
    lat_vert = [row[1] for row in coord_vert]

    # find the minumum/maximum longitude and latitude among the vertices
    min_lon = np.min(lon_vert)
    max_lon = np.max(lon_vert)
    min_lat = np.min(lat_vert)
    max_lat = np.max(lat_vert)

    # compute width and height of the rectangular shaped virtual map 
    position_width = CC.nav_to_local(min_lat, max_lon, min_lat, min_lon, R_earth)
    position_height = CC.nav_to_local(max_lat, min_lon, min_lat, min_lon, R_earth)

    map_width = position_width[1]
    map_height = position_height[0]

    # Compute the number of rows and columns of the matrix, basing on the resolution
    n_rows = int(map_height / res_meters) + 1
    n_columns = int(map_width / res_meters) + 1
    
    # Initialize the values of all the entries of the matrix at 0, which represents a free cell
    map_matrix = np.zeros((n_rows, n_columns), dtype=np.int32)

    scaled_map = np.zeros((n_rows, n_columns), dtype=np.int8)

    return map_matrix
 


def create_occupancy_grid(n_rows, n_cols, res_meters, grid, frame_id = "my_frame_id"):
    """
    
    This function creates and returns a OccupancyGrid message for visualization

    Arguments
     - grid: This is the grid representing the map. It must be passed 
             as a numpy ndarray with dtype=np.int8 (8-bit integers). 
             The elements of the grid must be 0 or 1, indicating 
             respectively free cell, or occupied cell.
     - res_meters: dimension of one grid cell, expressed in [m]
     - n_rows: number of rows of grid, this is also grid.shape[0]
     - frame_id : name of the frame we are referring this pose. By default this is "my_frame_id"

    Outputs
     - v_map: a OccupancyGrid() message, ready to be visualized
    
    """

    if grid is None:
        rospy.logerr("Grid is None. Cannot create occupancy grid.")
        return None

    if not isinstance(grid, np.ndarray):
        rospy.logerr("Grid must be a numpy ndarray.")
        return None

    if grid.dtype != np.int8:
        rospy.logerr("Grid dtype must be np.int8.")
        return None

    OG_map = OccupancyGrid()
    
    OG_map.header.stamp = rospy.Time.now()
    OG_map.header.frame_id = frame_id

    OG_map.info.map_load_time = rospy.Time.now()
    OG_map.info.resolution = res_meters
    OG_map.info.width = n_rows
    OG_map.info.height = n_cols
    
    OG_map.info.origin.position.x = 0.0
    OG_map.info.origin.position.y = 0.0
    OG_map.info.origin.position.z = 0.0

    OG_map.info.origin.orientation.x = 0.0
    OG_map.info.origin.orientation.y = 0.0
    OG_map.info.origin.orientation.z = 0.0
    OG_map.info.origin.orientation.w = 1.0
          
    grid_copy = grid.T
    OG_map.data = grid_copy.flatten().tolist()


    return OG_map


def conversion_map(vmap):
     # Put the values from 0 and 100 
     for i in range(n_rows):
        for j in range(n_columns):
            scaled_map[i,j] = int((vmap[i,j])*100/max_value_cell)
     


# -------------------------------------------------------------------------------------------------------
# CALLBACK
# -------------------------------------------------------------------------------------------------------


def map_callback(msg):
    
    #rospy.loginfo("Read the sensor data")

    zeno_lat = msg.latitude
    zeno_lon = msg.longitude
    zeno_yaw = msg.yaw

    #zeno_position_meters = CC.nav_to_local(zeno_lat, zeno_lon, min_lat, min_lon)
    [zeno_x, zeno_y] = CC.nav_to_local(zeno_lat, zeno_lon, min_lat, min_lon)

    zeno_position_meters = [zeno_x, zeno_y]

    alpha = math.pi/2 - zeno_yaw        # alpha staarts from the x axis and is positive counter-clockwise

    d_obs = msg.d_obs                   # array that defines if each beam found an obstacle

    angle_between_beams = angular_view / (number_of_beams - 1)

    beam_division = int(sonar_radius / res_meters)

    updated_cells = UpdatedCells()
    updated_cells.header.stamp = rospy.Time.now()
    updated_cells.dim = 0
    updated_cells.minlon = min_lon
    updated_cells.minlat = min_lat
    updated_cells.maxlon = max_lon
    updated_cells.maxlat = max_lat

    for k in range(number_of_beams):
        
        if d_obs[k]<0 and d_obs[k]!=-1:
            rospy.loginfo("ATTENTION: negative distance")
        beam_angle = alpha + (angular_view / 2) -  k * angle_between_beams  # this angle selects a different beam for every k

        # If the beam has not found an obstacle, every cell in that direction is selected as free
        # if a cell changes from not-free to free we add its coordinates to the list that has to be shared
        if d_obs[k] < 0:
            for p in range(1, (beam_division + 1)):
                d_free = p * res_meters
                x_free = zeno_position_meters[0] + d_free * math.sin(beam_angle)
                y_free = zeno_position_meters[1] + d_free * math.cos(beam_angle)

                i_free = int(x_free / res_meters)
                j_free = int(y_free / res_meters)

                if i_free < n_rows and i_free >= 0 and j_free < n_columns and j_free >= 0:

                    if virtual_map[i_free, j_free] == free_threshold:
                        virtual_map[i_free, j_free] = 0
                        updated_cells.row_list.append(i_free)        
                        updated_cells.column_list.append(j_free)
                        updated_cells.dim += 1
                    elif virtual_map[i_free, j_free] > free_threshold:
                        virtual_map[i_free, j_free] -= free_threshold
                    
        # If the beam has found an obstacle the cell at the communicated distance is set at the maximum value
        # the value of cells in the surroundings is changed to represent uncertainty
        # the cells before the obstacle in that direction are selected as free
        else:
            x_obs = zeno_position_meters[0] + d_obs[k] * math.sin(beam_angle)
            y_obs = zeno_position_meters[1] + d_obs[k] * math.cos(beam_angle)

            i_obs = int(x_obs / res_meters)
            j_obs = int(y_obs / res_meters)

            cells_margin = int(margin_meters / res_meters)

            for m in range(-cells_margin, (cells_margin + 1)):
                for n in range(-cells_margin, (cells_margin + 1)):
                    if (i_obs + m) < n_rows and (i_obs + m) >= 0 and (j_obs + n) < n_columns and (j_obs + n) >= 0:
                        if (m == 0 and n == 0):
                            if virtual_map[i_obs, j_obs] == 0:
                                updated_cells.row_list.append(i_obs)
                                updated_cells.column_list.append(j_obs)
                                updated_cells.dim += 1
                                virtual_map[i_obs, j_obs] = max_value_cell
                            else:
                                virtual_map[i_obs, j_obs] = max_value_cell
                        else:
                            if virtual_map[i_obs + m, j_obs + n] == 0:
                                virtual_map[i_obs + m, j_obs + n] = int(0.5*max_value_cell)
                                updated_cells.row_list.append(i_obs + m)
                                updated_cells.column_list.append(j_obs + n)
                                updated_cells.dim += 1
                            elif virtual_map[i_obs + m, j_obs + n] <= int(0.5*max_value_cell):
                                virtual_map[i_obs + m, j_obs + n] += int(0.5*max_value_cell)
                            else:
                                virtual_map[i_obs + m, j_obs + n] = max_value_cell

            
            obstacle_division = int(d_obs[k] / res_meters)
            for p in range(1, obstacle_division):
                d_free = p * res_meters
                x_free = zeno_position_meters[0] + d_free * math.sin(beam_angle)
                y_free = zeno_position_meters[1] + d_free * math.cos(beam_angle)

                i_free = int(x_free / res_meters)
                j_free = int(y_free / res_meters)

                if i_free < (n_rows - cells_margin) and i_free >= cells_margin and j_free < (n_columns - cells_margin) and j_free >= cells_margin:

                    if ((i_free != i_obs) and (i_free != i_obs + cells_margin) and (i_free != (i_obs - cells_margin)) or (j_free != j_obs) and (j_free != (j_obs + cells_margin)) and (j_free != (j_obs - cells_margin))):
                        if virtual_map[i_free, j_free] == free_threshold:
                            virtual_map[i_free, j_free] = 0
                            updated_cells.row_list.append(i_free)
                            updated_cells.column_list.append(j_free)
                            updated_cells.dim += 1
                        elif virtual_map[i_free, j_free] > free_threshold:
                            virtual_map[i_free, j_free] -= free_threshold
                        
    # The field of the message needs to be an array of int32, so we change it into that
    empty_row_list = np.empty(updated_cells.dim, dtype=np.uint32)
    empty_col_list = np.empty(updated_cells.dim, dtype=np.uint32)

    for i in range(updated_cells.dim):
        empty_row_list[i] = updated_cells.row_list[i]
        empty_col_list[i] = updated_cells.column_list[i]

    updated_cells.row_list = empty_row_list
    updated_cells.column_list = empty_col_list


    # publish
    #rospy.loginfo("Let's publish the updated cells")
    pub_cells.publish(updated_cells)




# -------------------------------------------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------------------------------------------


if __name__ == '__main__':

    virtual_map = initialize_map()

    try:
        rospy.init_node("v_map_generator", anonymous = False)

        pub_map = rospy.Publisher('/uncertain_map', OccupancyGrid, queue_size = 5)
        pub_cells = rospy.Publisher('/updated_cells', UpdatedCells, queue_size = 10)

        sub_sonar = rospy.Subscriber('/sonar_data', MySensor, callback = map_callback)

        
        rate_map = rospy.Rate(0.1)       # the whole map is published with a rate of 1 Hz

        while not rospy.is_shutdown():
            conversion_map(virtual_map)
            grid = create_occupancy_grid(n_rows, n_columns, res_meters, scaled_map)
        
            pub_map.publish(grid)           
            rate_map.sleep()
        

        rospy.spin() 

    except rospy.ROSInterruptException:
        pass
