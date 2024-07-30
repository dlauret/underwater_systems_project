#!/usr/bin/env python

import rospy
import numpy as np
# Imports for Visualization:
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
# Imports for A* algorithm
import math # type: ignore
# Custom Imports
import change_of_coordinates_func as cc
import grid_gen_func as gg
import rviz_util as ru
from A_star_func import A_star
# Imports for Zeno
from interact_with_zeno.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus 
# Imports for the subscription on updated_cells topic
from obstacle_avoidance.msg  import UpdatedCells
import read_file_map
from parameters import *
from ref_generation_func import ref_generation

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF WHAT SHOULD BE EXTRACTED FROM .TXT FILE
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Limits of map, expressed as latitude and longitude [_deg]
map_data = read_file_map.read_from_file()
row_indices, column_indices = zip(*map_data)

lat_min = min(column_indices)
lat_max = max(column_indices)

lon_min = min(row_indices)
lon_max = max(row_indices)


#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF INDEPENDENT PARAMETERS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Saturation values of references
err_yaw_max = 0.5   # [rad]
surge_max = 0.3     # [m/s]
# the surge_min value is a rosparam at the moment initialized to 0.02 m/s

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF DEPENDENT PARAMETERS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

# resolution in decimal degrees: dimension of one grid cell, expressed in decimal degrees

n_rows = int(np.ceil((lat_max - lat_min)/res_dec_deg))
n_cols = int(np.ceil((lon_max - lon_min)/res_dec_deg))

x_max = n_rows*res_meters
y_max = n_cols*res_meters

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF CLASS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

class Data_from_Topics:
    def __init__(self, start_lat, start_lon, start_yaw):
        self.nav_data = NavStatus()
        self.nav_data.position.latitude = start_lat
        self.nav_data.position.longitude = start_lon
        self.nav_data.orientation.yaw = start_yaw
        self.updated_cells = UpdatedCells()
        self.grid = gg.create_random_grid(lat_min, lat_max, lon_min, lon_max, res_dec_deg, res_meters, R_earth, 0)

    def nav_status_callback(self, nav_data):
        self.nav_data = nav_data

    def updated_cells_callback(self, updated_cells):
        self.updated_cells = updated_cells
        # Modify the grid based on the data received on updated_cells topic
        row_list = self.updated_cells.row_list
        column_list = self.updated_cells.column_list
        dim = self.updated_cells.dim
        for i in range(dim):
            row = row_list[i]
            col = column_list[i]
            self.grid[row, col] = not self.grid[row, col]

    def get_nav_data(self):
        return self.nav_data

    def get_updated_grid(self):
        return self.grid

#----------------------------------------------------------------------------------------------------------------------------------------------------------------    
# DEFINITION OF MAIN                
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def main():

    # Node initialization
    rospy.init_node('Path_generator')
    rate = rospy.Rate(2)

    # Create publishers
    pub_ref = rospy.Publisher('/relative_error', Rel_error_joystick, queue_size=10) 
    pub_goal = rospy.Publisher('/goal_position', PointStamped, queue_size=1)
    pub_path = rospy.Publisher('/way_points', Path, queue_size=1)
    
    # Start position
    start_latitude = 43.7066873
    start_longitude = 10.4754723
    start_yaw = np.random.uniform(-0.1, 1.0)
    
    # Node subscriptions
    data_from_topics = Data_from_Topics(start_latitude, start_longitude, start_yaw)
    rospy.Subscriber('/nav_status', NavStatus, data_from_topics.nav_status_callback)
    rospy.Subscriber('/updated_cells', UpdatedCells, data_from_topics.updated_cells_callback) 
    
    # initialize goal positions
    last_goal_latitude = last_goal_longitude = 0.0
    # Goal position:
    if not rospy.has_param('/goal_position'):   # this condition checks if the goal position position param is set to a value or not
        # If the goal_position is not initialized in the launch, then we random generate it inside the map of the .txt
        last_goal_latitude = goal_latitude = np.random.uniform(0.85, 0.90)*(lat_max - lat_min) + lat_min
        last_goal_longitude = goal_longitude = np.random.uniform(0.85, 0.90)*(lon_max - lon_min) + lon_min
        rospy.set_param('/goal_position', [goal_latitude, goal_longitude])
        [row_goal, col_goal] = cc.nav_to_grid(goal_latitude, goal_longitude, lat_min, lon_min, res_dec_deg)
    

    # Create grid 
    grid = gg.create_random_grid(lat_min, lat_max, lon_min, lon_max, res_dec_deg, res_meters, R_earth, 0)
    
    goal_found = 0 # counter to check if very near (1 cell) to the goal
    goal_changed = 0 # flag to check if the goal has changed
    #----------------------------------------------------------------------------

    while not rospy.is_shutdown():

        # Read Zeno position on nav_status topic and compute its location on the grid
        current_nav_data = data_from_topics.get_nav_data()
        current_latitude = current_nav_data.position.latitude
        current_longitude = current_nav_data.position.longitude

        ANGLE_INPUT = rospy.get_param('/ANGLE_INPUT')
        current_yaw = current_nav_data.orientation.yaw
        if ANGLE_INPUT == "deg":
            current_yaw = np.arctan2(np.sin(current_yaw*math.pi/180), np.cos(current_yaw*math.pi/180))
        else:
            current_yaw = np.arctan2(np.sin(current_yaw), np.cos(current_yaw))
        
        [row_start, col_start] = cc.nav_to_grid(current_latitude, current_longitude, lat_min, lon_min, res_dec_deg)
        start_yaw = current_yaw     # for the A_star algorithm, start is the current position each time it generates a path

        # Read updated Grid
        grid = data_from_topics.get_updated_grid()

        # Updating the goal based on the /goal_position param
        [goal_latitude, goal_longitude] = rospy.get_param('/goal_position')


        if goal_latitude != last_goal_latitude and goal_longitude != last_goal_longitude:
            print("WE HAVE A NEW GOAL: ", goal_latitude, goal_longitude)
            goal_changed = 1

        [row_goal, col_goal] = cc.nav_to_grid(goal_latitude, goal_longitude, lat_min, lon_min, res_dec_deg)
        [x_goal, y_goal] = cc.nav_to_local(goal_latitude, goal_longitude, lat_min, lon_min)

        last_goal_latitude, last_goal_longitude = goal_latitude, goal_longitude
        
        # Path planning
        if (abs(row_start-row_goal)+ abs(col_start-col_goal) ) > 2:   
            path, step = A_star(grid, (row_start, col_start), (row_goal, col_goal), start_yaw)
            goal_found = 0
        else:
            # very near the goal --> goal found
            path = None
            goal_found += 1
        

        # --------------------------------------------
        # Generate the references
        ref = Rel_error_joystick()

        [current_x, current_y] = cc.nav_to_local(current_latitude, current_longitude, lat_min, lon_min, R_earth)
        
        # Updating the rosparams
        surge_min = rospy.get_param('/surge_min')
        MODE_YAW = rospy.get_param('/MODE_YAW')
        MODE_SURGE = rospy.get_param('/MODE_SURGE')
        MODE_ZENO = rospy.get_param('/MODE_ZENO')
        N = rospy.get_param('/Number_of_WP')
        ANGLE_OUTPUT = rospy.get_param('/ANGLE_OUTPUT')

        # Calculating the references
        if path is not None:
            #d = 1   # if we are "d" waypoints away from the Goal then we stop Zeno
            ref.error_yaw, ref.error_surge_speed = ref_generation(path, N, current_x, current_y, current_yaw, err_yaw_max, surge_max, surge_min, MODE_YAW, MODE_SURGE, MODE_ZENO, ANGLE_OUTPUT)
        else: # is None
            if goal_found >= 1: # counter starts when Zeno is close to the goal
                if goal_found == 1:
                    # print just one time the message
                    print("ZENO HAS REACHED THE GOAL!!!: ", goal_latitude, goal_longitude)
                    goal_found += 1
                    goal_changed = 0
                # ref to 0 untill we have a new goal
                ref.error_yaw = ref.error_surge_speed = 0.0
                # if the goal has changed we restart the counter goal_found
                if goal_changed == 1:
                    goal_changed = 0
            else: # goal_found == 0
                # Zeno is inside an obstacle:
                print('ZENO IS INSIDE THE OBSTACLE OR GOAL OUTSIDE THE MAP or NOT FEASIBLE PATH FOUND')
                if MODE_ZENO == "stop":
                    err_yaw = 0.0
                    ref.error_surge_speed = 0.0
                else:
                    ref.error_surge_speed = 0.1 # [m/s]
                    angle = math.atan2(y_goal - current_y, x_goal - current_x)
                    err_yaw = angle - current_yaw
                    if err_yaw > err_yaw_max:
                        err_yaw = err_yaw_max

                    if err_yaw < -err_yaw_max:
                        err_yaw = -err_yaw_max
                    ref.error_yaw = err_yaw


                if ANGLE_OUTPUT == "deg":
                    ref.error_yaw = ref.error_yaw * 180/math.pi 

                ref.error_yaw = ref.error_yaw
        # --------------------------------------------
        # Publish references
        ref.header.stamp = rospy.Time.now()
        pub_ref.publish(ref)
        
        # --------------------------------------------
        # Visualization part 
        # --------------------------------------------

        # Goal Point
        [x_goal, y_goal] = cc.nav_to_local(goal_latitude, goal_longitude, lat_min, lon_min)
        goal_point = ru.point_create(x_goal, y_goal, 0.0)

        # Path
        rviz_path = ru.generate_WP(path, res_meters)

        # Publish visualization stuff
        pub_goal.publish(goal_point)
        pub_path.publish(rviz_path)

        # Wait for period
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    