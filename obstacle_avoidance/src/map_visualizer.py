#!/usr/bin/env python

import rospy
import numpy as np
# Imports for Visualization:
from nav_msgs.msg import OccupancyGrid
# Custom Imports
import grid_gen_func as gg
import rviz_util as ru
# Imports for the subscription on updated_cells topic
from obstacle_avoidance.msg  import UpdatedCells
import read_file_map
from parameters import *




class data_from_topics:
    def __init__(self):
        self.updated_cells = UpdatedCells()
        map_data = read_file_map.read_from_file()
        row_indices, column_indices = zip(*map_data)
        self.pub_vmap = rospy.Publisher('/v_map', OccupancyGrid, queue_size=1)

        self.lat_min = min(column_indices)
        self.lat_max = max(column_indices)
        self.lon_min = min(row_indices)
        self.lon_max = max(row_indices)
        self.n_rows = int(np.ceil((self.lat_max - self.lat_min)/res_dec_deg))
        self.n_cols = int(np.ceil((self.lon_max - self.lon_min)/res_dec_deg))

        self.grid = gg.create_random_grid(self.lat_min, self.lat_max, self.lon_min, self.lon_max, res_dec_deg, res_meters, R_earth, 0)

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
        
        occ_grid = ru.create_occupancy_grid(self.n_rows, self.n_cols, res_meters, self.grid)
        self.pub_vmap.publish(occ_grid)
        
def main():
    # Node initialization
    rospy.init_node('map_visualizer')

    # Initialize topic reader
    topic_reader = data_from_topics()

    # Create Subscribers
    rospy.Subscriber('/updated_cells', UpdatedCells, topic_reader.updated_cells_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Here you can include any necessary cleanup or shutdown tasks that should be performed before the node exits.
        pass