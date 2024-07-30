#!/usr/bin/env python

import rospy
import numpy as np
from obstacle_avoidance.msg import MySensor  # Import your custom message type
from sensor_msgs.msg import Range, LaserScan
from parameters import *


class Data_from_Topics:

    def __init__(self):
        self.sonar_data = MySensor()
        self.pub_laser = rospy.Publisher('/sonar_range_laser', LaserScan, queue_size=1)
        self.pub_range = rospy.Publisher('/sonar_range', Range, queue_size=1)


    def sonar_data_callback(self, sonar_data):
        self.sonar_data = sonar_data

        # Fill LaserScan Message

        laser_to_pub = LaserScan()
        laser_to_pub.header.stamp = sonar_data.header.stamp
        laser_to_pub.header.frame_id = "sonar_frame"

        laser_to_pub.angle_min = -angular_view/2
        laser_to_pub.angle_max = angular_view/2
        laser_to_pub.angle_increment = angular_view/number_of_beams

        laser_to_pub.time_increment = 0.0
        laser_to_pub.scan_time = 0.0

        laser_to_pub.range_min = 0.0
        laser_to_pub.range_max = sonar_radius*1.1

        ranges = np.ones(len(sonar_data.d_obs))
        for i in range(len(sonar_data.d_obs)):
            if sonar_data.d_obs[i] == -1.0: ranges[i] = sonar_radius
            else: ranges[i] = sonar_data.d_obs[i]

        laser_to_pub.ranges = ranges


        # Fill Range message
        range_to_pub = Range()

        range_to_pub.header.stamp = sonar_data.header.stamp
        range_to_pub.header.frame_id = "sonar_frame"

        range_to_pub.radiation_type = Range.ULTRASOUND

        range_to_pub.field_of_view = angular_view

        range_to_pub.min_range = 0.0
        range_to_pub.max_range = sonar_radius*1.1

        range_to_pub.range = sonar_radius

        # Publish messages

        self.pub_laser.publish(laser_to_pub)
        self.pub_range.publish(range_to_pub)



    def get_sonar_data(self):
        return self.sonar_data


def main():

    # Node initialization
    rospy.init_node('range_to_rviz')

    # Subscriber
    data_from_topics = Data_from_Topics()
    rospy.Subscriber('/sonar_data', MySensor, data_from_topics.sonar_data_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

