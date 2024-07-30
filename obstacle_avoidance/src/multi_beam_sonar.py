#!/usr/bin/env python

import rospy
import numpy as np
from shapely.geometry import Point, Polygon
import math
from obstacle_avoidance.msg import MySensor  # Import your custom message type
from marta_msgs.msg import NavStatus
from parameters import *

# Initialize variables to store polygon, circle, and map data
mappa = []
obstacles = []

def read_from_file():
    # Nome del file da leggere
    nome_file = "map_file.txt"

    # Open the text file
    with open(nome_file, 'r') as file:
        lines = file.readlines()
    
    current_polygon = None
    map_data = []
    # Iterate through each line in the file
    for index, line in enumerate(lines):
        # Strip leading and trailing whitespace from the line
        line = line.strip()

        # Extract map data from the first line
        if index == 0:
            if line.startswith("map"):
                map_data = eval(line.split('=')[1].strip())
                mappa.append(map_data)
                #print("map data: ",map_data)
                continue  # Move to the next line

        # Check if the line defines a polygon
        if line.startswith("polygon"):
            # If current_polygon is not None, append it to polygons list
            if current_polygon:
                obstacles.append(current_polygon)
            # Start a new polygon
            current_polygon = []
            # Extract coordinates from the line
            coordinates = eval(line.split('=')[1].strip())
            current_polygon.extend(coordinates)

        # Check if the line defines a circle
        elif line.startswith("circle"):
            # Extract center and radius from the line
            center, radius = eval(line.split('=')[1].strip())
            radius_deg = radius / 111000
            # Create circle vertices 
            # Create the circle obstacle using Shapely's buffer method
            circle_obstacle = Point(center).buffer(radius_deg)
            # Convert the circle obstacle to a list of its vertices
            polygon_vertices3 = list(circle_obstacle.exterior.coords)
            obstacles.append(polygon_vertices3)


    # Append the last polygon if it exists
    if current_polygon:
        obstacles.append(current_polygon)


def calculate_distance_array(longitude, latitude, yaw):

    cur_lon = longitude
    cur_lat = latitude
    cur_yaw = yaw

    map = Polygon(mappa[0])
    
    theta_min = cur_yaw - angular_view/2 # -60deg
    theta_max = cur_yaw + angular_view/2 # +60deg

    vector_theta_loop = np.linspace(theta_min, theta_max, number_of_beams) # 10 values evenly spaced from 30 to 150 deg
    vector_distance_loop = np.linspace(0, sonar_radius, 100)  # 100 values evenly spaced from 0 to sonar_radius meters

    distance_array = np.ones(number_of_beams) * (-1)  # Initialize
    index_theta = 0
    
    # loop for the theta (beam)
    for theta in vector_theta_loop:
        if distance_array[index_theta] == -1:
            # loop for the distance from 0 to sonar_radius
            for d in vector_distance_loop:
                if distance_array[index_theta] == -1:
                    
                    
                    rotated_theta = math.pi/2 - theta
                    dx = d*np.cos(rotated_theta) # [m]
                    dy = d*np.sin(rotated_theta) # [m]

                    new_longitude = cur_lon + (dx / R_earth) * (180 / math.pi) # more complex: / np.cos(latitude * math.pi/180)
                    new_latitude  = cur_lat  + (dy / R_earth) * (180 / math.pi)
                    # point to check: robot position + distance
                    point_i = Point(new_longitude, new_latitude)

                    # check if the point is outside the MAP
                    is_within_map = point_i.within(map)
                    #print(is_within_map)
                    if is_within_map == False:
                        #print("Mappa beam: ", index_theta)
                        distance_array[index_theta] = round(d + np.random.normal(0, 0.01), 3)

                    i_obs = -1
                    # check if the point is inside the obstacles (loop)
                    for obs in obstacles:
                        i_obs += 1
                        if distance_array[index_theta] == -1:
                            obstacle = Polygon(obs)
                            is_within_obstacle = point_i.within(obstacle)         
                            if is_within_obstacle:
                                distance_array[index_theta] = round(d + np.random.normal(0, 0.01), 3)
                        else:
                            break
                else:
                    break
        # increment the index of theta
        index_theta += 1
    
    return distance_array, cur_lon, cur_lat, cur_yaw

def callback_longitude_latitude_yaw(data):
    global longitude, latitude, yaw
    # Callback function to process longitude and latitude from the subscribed topic
    longitude = data.position.longitude
    latitude = data.position.latitude
    yaw = data.orientation.yaw
    #print(longitude, latitude, yaw)
    


if __name__ == '__main__':
    global longitude, latitude, yaw

    # read from txt file
    read_from_file()

    # Initialize the ROS node
    rospy.init_node('multi_beam_sonar', anonymous=False)

    # Define the publisher for the distance array
    pub = rospy.Publisher('/sonar_data', MySensor, queue_size=5)

    # Subscribe to the topic for longitude and latitude
    rospy.Subscriber('/nav_status', NavStatus, callback_longitude_latitude_yaw)
    
    rate = rospy.Rate(2) #2 Hz

    # global longitude, latitude, yaw

    rospy.sleep(1)

    while not rospy.is_shutdown():
        
        # Calculate the distance array
        distance_array, cur_lon, cur_lat, cur_yaw = calculate_distance_array(longitude, latitude,yaw)
    
        # Publish in sonar_data
        distance_msg = MySensor()
        distance_msg.header.stamp = rospy.Time.now()
        # distance_msg.num_beams = number_of_beams
        # distance_msg.range_radius =  sonar_radius
        #distance_msg.angular_view = angular_view
        distance_msg.longitude = cur_lon
        distance_msg.latitude = cur_lat
        distance_msg.yaw = cur_yaw
        distance_msg.d_obs = distance_array.tolist()


        # Publish the distance array
        pub.publish(distance_msg)
        #print("Should have published")

        # Sleep to maintain the publishing rate
        rate.sleep()

    # Spin
    rospy.spin()
