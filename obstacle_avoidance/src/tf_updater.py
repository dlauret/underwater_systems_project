#!/usr/bin/env python

import rospy
import numpy as np
import math 
# Imports for Visualization:
from geometry_msgs.msg import PoseStamped
# Custom Imports
import change_of_coordinates_func as cc
import rviz_util as ru
# Interacting with Zeno Imports
from interact_with_zeno.msg import Rel_error_joystick
from marta_msgs.msg import NavStatus
import read_file_map
from parameters import *
from obstacle_avoidance.msg import MySensor
from visualization_msgs.msg import Marker

class data_from_topics:
    def __init__(self):
        self.nav_data = NavStatus()
        self.sonard_data = MySensor()
        self.pub_zeno = rospy.Publisher('/zeno_pose', PoseStamped, queue_size=1)
        #-----------------------------------------------------------------
        # Mesh di Zeno:
        self.pub_mesh = rospy.Publisher('/zeno_mesh', Marker, queue_size=1)
        #-----------------------------------------------------------------
        map_data = read_file_map.read_from_file()
        row_indices, column_indices = zip(*map_data)

        self.lat_min = min(column_indices)
        self.lon_min = min(row_indices)

    def nav_status_callback(self, nav_data):
        self.nav_data = nav_data
        [zeno_x, zeno_y] = cc.nav_to_local(nav_data.position.latitude, nav_data.position.longitude,
                                              self.lat_min, self.lon_min)
        ru.transform_broadcast("map", "my_frame_id", 0, 0, 0, 0, math.pi, -math.pi/2)
        ru.transform_broadcast("my_frame_id", "body_frame", zeno_x, zeno_y, 0, 0, 0, nav_data.orientation.yaw)
        zeno_pose = ru.pose_create(zeno_x, zeno_y, 0.0, 0.0, 0.0, nav_data.orientation.yaw)
        self.pub_zeno.publish(zeno_pose)
        #-----------------------------------------------------------------
        # Mesh di Zeno
        marker = Marker()
        marker.header.frame_id = "body_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = -1.0
        marker.pose.position.y = -0.73
        marker.pose.position.z = -0.80
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.003
        marker.scale.y = 0.003
        marker.scale.z = 0.003
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.mesh_resource = "package://obstacle_avoidance/src/meshes/MESH_ZENO_4.STL"
        self.pub_mesh.publish(marker)
        #-----------------------------------------------------------------


    def sonar_data_callback(self, sonar_data):
        self.sonar_data = sonar_data
        [sonar_x, sonar_y] = cc.nav_to_local(sonar_data.latitude, sonar_data.longitude,
                                              self.lat_min, self.lon_min)

        ru.transform_broadcast("my_frame_id", "sonar_frame", sonar_x, sonar_y, 0.0,
                               0.0, 0.0, sonar_data.yaw)

    def get_nav_data(self):
        return self.nav_data
    

def main():
    # Node initialization
    rospy.init_node('tf_updater')

    # Initialize topic reader
    topic_reader = data_from_topics()

    # Create Subscribers
    rospy.Subscriber('/sonar_data', MySensor, topic_reader.sonar_data_callback)
    rospy.Subscriber('/nav_status', NavStatus, topic_reader.nav_status_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Here you can include any necessary cleanup or shutdown tasks that should be performed before the node exits.
        pass