import numpy as np
import math
from parameters import *

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# CHANGE OF COORDINATES FUNCTIONS
#----------------------------------------------------------------------------------------------------------------------------------------------------------------

def nav_to_grid(lat, lon, lat_min, lon_min, res_dec_deg):
    """

    This functions takes as input the latitude and longitude
    of a point expressed in decimal degrees, and returns its
    coordinates in the grid representing the map.

    Arguments
     - lat: latitude of the point, expressed in [_ deg]
     - lon: longitude of the point, expressed in [_ deg]
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - res_dec_deg: resolution of a cell, expressed in [_ deg]

    Outputs
     - [row, col]: coordinates of the point in the grid, expressed in []
    
    """
    row = np.floor((lat - lat_min)/res_dec_deg)
    col = np.floor((lon - lon_min)/res_dec_deg)

    return [row, col]

#----------------------------------------------------------- 

def grid_to_nav(row, col, lat_min, lon_min, res_dec_deg):
    """

    This functions takes as input the coordinates of a point 
    in the grid representing the map and returns its
    latitude and longitude expressed in decimal degrees.

    Arguments
     - row: row of the point in the grid, expressed in []
     - col: cell of the point in the grid, expressed in []
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - res_dec_deg: resolution of a cell, expressed in [_ deg]

    Outputs
     - [lat, lon]: latitude and longitude of the point, expressed in [_ deg]
    
    """
    lat = (row + 0.5)*res_dec_deg + lat_min
    lon = (col + 0.5)*res_dec_deg + lon_min

    return [lat, lon]

#----------------------------------------------------------- 

def nav_to_local(lat, lon, lat_min, lon_min, R_earth=6368000):
    """

    This functions takes as input the latitude and longitude
    of a point expressed in decimal degrees, and returns its
    coordinates in a local frame, expressed in meters.

    The local frame has the origin placed in (lat_min, lon_min),
    x-axis facing North, y-axis facing East, z-axis facing Down.

    Arguments
     - lat: latitude of the point, expressed in [_ deg]
     - lon: longitude of the point, expressed in [_ deg]
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - R_earth: radius of earth at local altitude, expressed in [m]
                by default, this is set to 6368000 [m]

    Outputs
     - [x, y]: coordinates of the point in the local frame, expressed in [m]
    
    """
    x = (lat - lat_min)*math.pi*R_earth/180
    y = (lon - lon_min)*math.pi*R_earth/180

    return [x, y]

#----------------------------------------------------------- 

def local_to_nav(x, y, lat_min, lon_min, R_earth=6368000):
    """

    This functions takes as input the coordinates of a point
    in the local frame expressed in meters, and returns its
    latitude and longitude, expressed in decimal degrees.

    The local frame has the origin placed in (lat_min, lon_min),
    x-axis facing North, y-axis facing East, z-axis facing Down.

    Arguments
     - x: x of the point in local frame, expressed in [m]
     - y: y of the point in local frame, expressed in [m]
     - lat_min: latitude lower limit of map, expressed in [_ deg]
     - lon_min: longitude lower limit of map, expressed in [_ deg]
     - R_earth: radius of earth at local altitude, expressed in [m]
                by default, this is set to 6368000 [m]

    Outputs
     - [lat, lon]: latitude and longitude of the point, expressed in [_ deg]
    
    """
    
    lat = x*180/(R_earth*math.pi) + lat_min

    lon = y*180/(R_earth*math.pi) + lon_min

    return [lat, lon]

#----------------------------------------------------------- 

def local_to_grid(x, y, res_meters):
    """

    This functions takes as input the coordinates of a point
    in the local frame expressed in meters, and returns its
    coordinates in the grid representing the map.

    The local frame has the origin placed in (lat_min, lon_min),
    x-axis facing North, y-axis facing East, z-axis facing Down.

    Arguments
     - x: x of the point in local frame, expressed in [m]
     - y: y of the point in local frame, expressed in [m]
     - res_meters: dimension of one grid cell, expressed in [m]

    Outputs
     - [row, col]: coordinates of the point in the grid, expressed in []
    
    """
    
    row = np.floor(x/res_meters)

    col = np.floor(y/res_meters)

    return [row, col]

#----------------------------------------------------------- 

def grid_to_local(row, col, res_meters):
    """

    This functions takes as input the coordinates of a point 
    in the grid representing the map, and returns its
    coordinates in a local frame, expressed in meters.

    The local frame has the origin placed in (lat_min, lon_min),
    x-axis facing North, y-axis facing East, z-axis facing Down.

    Arguments
     - row: x of the point in local frame, expressed in [m]
     - col: y of the point in local frame, expressed in [m]
     - res_meters: dimension of one grid cell, expressed in [m]

    Outputs
     - [x, y]: coordinates of the point in the grid, expressed in []
    
    """
    
    x = (row + 0.5)*res_meters

    y = (col + 0.5)*res_meters

    return [x, y]