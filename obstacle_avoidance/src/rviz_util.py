#!/usr/bin/env python

import numpy as np
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from collections import deque

import change_of_coordinates_func as cc

#--------------------------------------------------------------
#   Function that creates and broadcast a roto-traslational transform between frames
#--------------------------------------------------------------

def transform_broadcast(frame_1, frame_2, dx, dy, dz, roll, pitch, yaw):

    """
    
    This function creates and broadcasts a roto-translational transform between 
    frame_1 and frame_2. This is necessary for proper rviz visualization.

    Arguments
     - frame_1 : name of the starting frame, passed as a string
     - frame_2 : name of the arrival frame, passed as a string
     - dx, dy, dz : traslational part of transform, expressed in [m]
     - roll, pitch, yaw : rotational part of transform, expressed in [rad]

    Outputs
     - This yelds no output

    Typical usage
     - Since Rviz "map" frame is a ENU frame, and local frame is tipycally a NED
       frame, we'd like to broadcast a transform that maps from ENU to NED:

       import rviz_util as ru
       ru.transform_broadcast("map", "my_frame_id", 0, 0, 0, 0, math.pi, -math.pi/2)
    
    """

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = frame_1
    static_transformStamped.child_frame_id = frame_2

    static_transformStamped.transform.translation.x = dx
    static_transformStamped.transform.translation.y = dy
    static_transformStamped.transform.translation.z = dz

    q = quaternion_from_euler(roll, pitch, yaw)

    static_transformStamped.transform.rotation.x = q[0]
    static_transformStamped.transform.rotation.y = q[1]
    static_transformStamped.transform.rotation.z = q[2]
    static_transformStamped.transform.rotation.w = q[3]

    broadcaster.sendTransform(static_transformStamped)


#--------------------------------------------------------------
#   Function that creates and returns a PoseStamped message
#--------------------------------------------------------------

def pose_create(x, y, z, roll, pitch, yaw, frame_id = "my_frame_id"):
    """
    
    This function creates and returns a PoseStamped() message from given
    coordinates and roll-pitch-yaw angles.

    Arguments
     - x, y, z : position of the object with respect to the "frame_id" frame
     - roll, pitch, yaw : orientation of the object with respect to the "frame_id" frame
     - frame_id : name of the frame we are referring this pose. By default this is "my_frame_id"
    
    Outputs
     - mypose: a PoseStamped() message with all fields covered, ready to be visualized.
    
    """

    mypose = PoseStamped()
    mypose.header.frame_id = frame_id
    mypose.header.stamp = rospy.Time.now()

    mypose.pose.position.x = x
    mypose.pose.position.y = y
    mypose.pose.position.z = z

    q = quaternion_from_euler(roll, pitch, yaw)

    mypose.pose.orientation.x = q[0]
    mypose.pose.orientation.y = q[1]
    mypose.pose.orientation.z = q[2]
    mypose.pose.orientation.w = q[3]

    return mypose

#--------------------------------------------------------------
#   Function that creates and returns a PointStamped message
#--------------------------------------------------------------

def point_create(x, y, z, frame_id = "my_frame_id"):
    """
    
    This function creates and returns a PointStamped() message from given
    coordinates.

    Arguments
     - x, y, z : position of the object with respect to the "frame_id" frame
     - frame_id : name of the frame we are referring this pose. By default this is "my_frame_id"
    
    Outputs
     - mypoint: a PointStamped() message with all fields covered, ready to be visualized.
    
    """

    mypoint = PointStamped()
    mypoint.header.frame_id = frame_id
    mypoint.header.stamp = rospy.Time.now()

    mypoint.point.x = x
    mypoint.point.y = y
    mypoint.point.z = z

    return mypoint

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# FROM PATH TO PHYSICAL WAYPOINTS FUNCTION
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def generate_WP(reconstructed_path, res_meters, frame_id = "my_frame_id"):
    """
    
    This function converts the path returned by A_star algorithm into
    a sequence of waypoints in the local frame. These waypoints are then 
    used to create an instance of a Path message.

    Inputs
     - reconstructed_path: path returned by A_star algorithm, passed as
                           a deque object

     - res_meters: dimension of one grid cell, expressed in [m]
     - frame_id : name of the frame we are referring this pose. By default this is "my_frame_id"

    Outputs
     - path: this is a standard navigation message of type Path(), which
             can be displayed by RVIZ. 
    
    Typical usage
     - rviz_path = generate_WP(A_star_path, res_meters)
    
    """
    # Path type object inizialization
    path = Path()   
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()

    list = reconstructed_path

    while list:

        # Extracting the first element from A* path
        cell = list.popleft()
        
        # PoseStamped type object inizialization
        way_point = PoseStamped()

        # Inserting the physical position of the way points:
        [x, y] = cc.grid_to_local(cell[0], cell[1], res_meters)
        way_point.pose.position.x = x
        way_point.pose.position.y = y
        way_point.pose.position.z = 0

        # Iteratively creating the Path
        path.poses.append(way_point)

    return path  

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# FROM GRID TO OccupancyGrid FUNCTION
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

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

    v_map = OccupancyGrid()

    v_map.header.stamp = rospy.Time.now()
    v_map.header.frame_id = frame_id

    v_map.info.map_load_time = rospy.Time.now()
    v_map.info.resolution = res_meters
    v_map.info.width = n_rows
    v_map.info.height = n_cols
    
    v_map.info.origin.position.x = 0.0
    v_map.info.origin.position.y = 0.0
    v_map.info.origin.position.z = 0.0

    v_map.info.origin.orientation.x = 0.0
    v_map.info.origin.orientation.y = 0.0
    v_map.info.origin.orientation.z = 0.0
    v_map.info.origin.orientation.w = 1.0
          
    grid_copy = grid.T*100
    v_map.data = grid_copy.flatten().tolist()


    return v_map    

