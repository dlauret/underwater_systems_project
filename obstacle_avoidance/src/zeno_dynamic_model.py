#!/usr/bin/env python

import rospy
import math 
import numpy as np
# Imports for Visualization:
from geometry_msgs.msg import PoseStamped
# Custom Imports
#from Utils import change_of_coordinates_func as cc
import change_of_coordinates_func as cc
#from Utils import rviz_util as ru
import rviz_util as ru
# Interacting with Zeno Imports
from interact_with_zeno.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus
# Utils
#from Utils import read_file_map
import read_file_map
from parameters import *

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
""" NODE DESCRIPTION """
# This node simulates the dynamics of Zeno and his autopilot
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
""" NOMOTO MODEL: """
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# Parameters: (Datasheet + assumptions)
M = 44      # [kg]
X_u = 1     # [N*s/m]   Linear Damping coefficient  
I = 6.4     # [kg*m^2]  Monent of Inertia along z body axis , estimated with the formula 1/12*M*(1.030^2 + 0.830^2) assuming Zeno as a cuboid

# Dynamic Equations:
#   1) M*u_dot + X_u*u = tau_u
#   2) I*r_dot + r = tau_yaw
# where u = surge and r = yaw_dot 
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
""" MAP LIMITS """
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF WHAT SHOULD BE EXTRACTED FROM .TXT FILE

# Limits of map, expressed as latitude and longitude [_deg]
map_data = read_file_map.read_from_file()
row_indices, column_indices = zip(*map_data)
lat_min = min(column_indices)
lat_max = max(column_indices)

lon_min = min(row_indices)
lon_max = max(row_indices)

x_max = cc.nav_to_local(lat_max, lon_max, lat_min, lon_min)[0]
y_max = cc.nav_to_local(lat_max, lon_max, lat_min, lon_min)[1]

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF CLASS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
class data_from_topics:
    def __init__(self, start_lat, start_lon, start_yaw):
        self.nav_data = NavStatus()
        self.nav_data.position.latitude = start_lat
        self.nav_data.position.longitude = start_lon
        self.nav_data.orientation.yaw = start_yaw
        self.rel_err_joy = Rel_error_joystick()
        self.yaw_ref = 0.0

    def nav_status_callback(self, nav_data):
        self.nav_data = nav_data

    def ref_callback(self, rel_err_joy):
        self.rel_err_joy = rel_err_joy    
        self.rel_err_joy.error_yaw = self.rel_err_joy.error_yaw * math.pi/180    # converting the error_yaw from [deg] to [rad]    
        self.yaw_ref = math.atan2(math.sin(rel_err_joy.error_yaw + self.nav_data.orientation.yaw), math.cos(rel_err_joy.error_yaw + self.nav_data.orientation.yaw))

    def get_nav_data(self):
        return self.nav_data

    def get_rel_err_joy(self):
        return self.rel_err_joy
    
    def get_yaw_ref(self):
        return self.yaw_ref
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF MAIN
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def main():
    # Node initialization
    rospy.init_node('zeno_model')

    # Create publishers
    pub_nav_status = rospy.Publisher('/nav_status', NavStatus, queue_size=1)

    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    """ Initial Position and heading of Zeno """
    # Feasible random start position:
    #start_lat = 0.7*(lat_max - lat_min) + lat_min
    #start_lon = 0.1*(lon_max - lon_min) + lon_min
    #start_yaw = np.random.uniform(-0.3, 2.0)


    # Partenza Obstacle_Avoidance Hard Experiment:
    start_lat = 43.7065389067
    start_lon = 10.4752151903
    start_yaw = np.random.uniform(-0.1, 1.0)
    #------------------------------------------------------------------------------------------------------------------------------------------------------------

    # Initialize topic reader
    topic_reader = data_from_topics(start_lat, start_lon, start_yaw)

    # Create Subscribers
    rospy.Subscriber('/relative_error', Rel_error_joystick, topic_reader.ref_callback)
    rospy.Subscriber('/nav_status', NavStatus, topic_reader.nav_status_callback)
    
    # Initialize messages to publish
    nav_status = topic_reader.get_nav_data()

    # Publishing Rate
    T_s = 0.1                  # Euler (EA) integration fixed step (for dynamics)
    rate = rospy.Rate(1/T_s)    # Publishing at each integration step

    
    """ Discretize Dynamics Initialization """
    r_zeno = 0                 # yaw rate at k=0 step
    error_yaw_integral = 0  # integral over time of yaw error at k=0 step
    u_zeno = 0
    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    """ Main Loop """

    while not rospy.is_shutdown():

        # Read Zeno position on nav_status topic
        cur_nav_data = topic_reader.get_nav_data()
        cur_latitude = cur_nav_data.position.latitude
        cur_longitude = cur_nav_data.position.longitude
        [x_zeno, y_zeno] = cc.nav_to_local(cur_latitude, cur_longitude, lat_min, lon_min)
        cur_yaw = cur_nav_data.orientation.yaw
        cur_yaw = math.atan2(math.sin(cur_yaw), math.cos(cur_yaw))

        # Read References
        cur_ref = topic_reader.get_rel_err_joy()
        ref_surge = cur_ref.error_surge_speed
        #ref_error_yaw = cur_ref.error_yaw

        # Computed References
        yaw_ref = topic_reader.get_yaw_ref()

        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        """ PID Controller for Heading """
        # Selecting the PID coefficients:
        K_yaw_P = 30        # [N*m/rad]
        K_yaw_I = 0.1       # [N*m/(s*rad)]
        K_yaw_D = 30        # [N*m*s/rad]

        # Calculating the heading error from the feedback loop:
        error_yaw = yaw_ref - cur_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))    # remapping

        # Controller output:
        tau_yaw = + K_yaw_P * error_yaw + K_yaw_I * error_yaw_integral - K_yaw_D * r_zeno  # PID controller with anti-kicking for the derivative term
        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        """ PI Controller for surge """
        # Selecting the PI coefficients:
        K_u_P = 100           # [N*s/m]       
        # K_u_I = 0.0          # [N/m]

        # Calculating the surge error from the feedback loop:
        error_surge = ref_surge - u_zeno
        # Controller output:
        tau_u = + K_u_P * error_surge
        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        """ Updating the yaw_error_integral """

        error_yaw_integral = error_yaw_integral + T_s * error_yaw
        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        """ Zeno Discrete Dynamics """
        
        # Calculating u_(k+1) and r_(k+1):
        u_zeno = u_zeno - (T_s/M)*X_u*u_zeno + (T_s/M)*tau_u        
        r_zeno = r_zeno - (T_s/I)*r_zeno + (T_s/I)*tau_yaw

        # Calculating yaw_(k+1) , x_(k+1) and y_(k+1)
        if cur_yaw > 2*math.pi or cur_yaw < -2*math.pi:
            cur_yaw = (cur_yaw + T_s*r_zeno) % (2*math.pi)
        else:
            cur_yaw = (cur_yaw + T_s*r_zeno)

        cur_yaw = math.atan2(math.sin(cur_yaw), math.cos(cur_yaw))  #remapping   

        x_zeno = x_zeno + T_s*u_zeno*math.cos(cur_yaw)
        y_zeno = y_zeno + T_s*u_zeno*math.sin(cur_yaw)
        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        # Update nav_status
        [cur_latitude, cur_longitude] = cc.local_to_nav(x_zeno, y_zeno, lat_min, lon_min)
        nav_status.position.latitude = cur_latitude
        nav_status.position.longitude = cur_longitude
        nav_status.orientation.yaw = cur_yaw

        # Publish 
        nav_status.header.stamp = rospy.Time.now()
        pub_nav_status.publish(nav_status)

        rate.sleep()
        #--------------------------------------------------------------------------------------------------------------------------------------------------------
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Here you can include any necessary cleanup or shutdown tasks that should be performed before the node exits.
        pass

