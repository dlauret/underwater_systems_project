#!/usr/bin/env python

import rospy
import numpy as np
# Imports for Visualization:
from geometry_msgs.msg import PoseStamped
# Custom Imports
import change_of_coordinates_func as cc
# Interacting with Zeno Imports
from interact_with_zeno.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus
import read_file_map
from parameters import *

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF WHAT SHOULD BE EXTRACTED FROM .TXT FILE
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Limits of map, expressed as latitude and longitude [_deg]

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

    def nav_status_callback(self, nav_data):
        self.nav_data = nav_data

    def ref_callback(self, rel_err_joy):
        self.rel_err_joy = rel_err_joy

    def get_nav_data(self):
        return self.nav_data

    def get_rel_err_joy(self):
        return self.rel_err_joy
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# DEFINITION OF MAIN
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def main():
    # Node initialization
    rospy.init_node('Zeno_Model')

    # Create publishers
    pub_nav_status = rospy.Publisher('/nav_status', NavStatus, queue_size=1)

    # The Euler integration is at fixed step T_s, at the moment each time the integration and publishing rate are the same (we publish all the steps)
    T_s = 0.1
    rate = rospy.Rate(1/T_s)

    # Create random start pose
    start_lat = 43.7066873
    start_lon = 10.4754723
    start_yaw = np.random.uniform(-0.1, 1.0)

    # Initialize topic reader
    topic_reader = data_from_topics(start_lat, start_lon, start_yaw)

    # Create Subscribers
    rospy.Subscriber('/relative_error', Rel_error_joystick, topic_reader.ref_callback)
    rospy.Subscriber('/nav_status', NavStatus, topic_reader.nav_status_callback)

    # Initialize messages to publish
    nav_status = topic_reader.get_nav_data()

    # Compute gain on yaw dynamics
    k_yaw = 0.15

    while not rospy.is_shutdown():
        
        # Read Zeno position on nav_status topic
        cur_nav_data = topic_reader.get_nav_data()
        cur_latitude = cur_nav_data.position.latitude
        cur_longitude = cur_nav_data.position.longitude
        cur_yaw = cur_nav_data.orientation.yaw
        cur_yaw = np.arctan2(np.sin(cur_yaw), np.cos(cur_yaw))

        # Read References
        cur_ref = topic_reader.get_rel_err_joy()
        ref_surge = cur_ref.error_surge_speed
        ref_error_yaw = cur_ref.error_yaw

        # Compute new Angle
        cur_yaw = cur_yaw + k_yaw*ref_error_yaw*T_s
        

        # Compute new Position 
        [x_zeno, y_zeno] = cc.nav_to_local(cur_latitude, cur_longitude, lat_min, lon_min)
        x_zeno = x_zeno + ref_surge*T_s*np.cos(cur_yaw)
        y_zeno = y_zeno + ref_surge*T_s*np.sin(cur_yaw)
        
        # Update nav_status
        [cur_latitude, cur_longitude] = cc.local_to_nav(x_zeno, y_zeno, lat_min, lon_min)
        nav_status.position.latitude = cur_latitude
        nav_status.position.longitude = cur_longitude
        nav_status.orientation.yaw = cur_yaw

        # Publish 
        nav_status.header.stamp = rospy.Time.now()
        pub_nav_status.publish(nav_status)

        rate.sleep()

        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Here you can include any necessary cleanup or shutdown tasks that should be performed before the node exits.
        pass

