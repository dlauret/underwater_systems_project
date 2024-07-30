## This file contains the parameters necessary for all the nodes
import math
import numpy
 
res_meters = 0.5                  # resolution in meters: dimension of one grid cell, expressed in [m]

R_earth = 6368000                 # radius of earth at (43_ N) latitude, expressed in [m]
            
number_of_beams = 15          # number of the beams of the sonar   

sonar_radius = 10                 # distance range of the sonar beams

angular_view = 120 * math.pi/180  # angular range of the sonar

res_dec_deg = 180 * res_meters/(math.pi * R_earth)  # resolution of a cell in terms of degree

margin_meters = numpy.max([res_meters, 1.0])    # if an obstacle is found, we consider obstacles also the cells around it in this radius [m]