import numpy
import math
import heapq
from collections import deque


def A_star(grid, start, goal, yaw):
    """
    
    This function produces a path that goes from start to goal. 
    The path is computed on a discrete grid representing the map,
    and it's main purpose is to avoid the obstacles.

    Arguments
        - grid: This is the grid representing the map. It must be passed 
                as a numpy ndarray with dtype=np.int8 (8-bit integers). 
                The elements of the grid must be 0 or 1, indicating 
                respectively free cell, or occupied cell.

        - start: Coordinates of the start of path (current position of Zeno)
                expressed as a python tuple composed by (row_start, col_start)

        - goal: Coordinates of the goal of path (target position)
                expressed as a python tuple composed by (row_goal, col_goal)

        - yaw: Current yaw angle of Zeno, this must be a float, expressed in [rad]

    Outputs
        - path: Resulting path, returned as a deque object

        - step: Number of steps needed to produce the path

    Usage
        - path, step = A_star(grid, start, goal, yaw)
        
    """
    def heuristic(a, b):
        return math.sqrt(((a[0] - b[0])**2) + ((a[1] - b[1])**2))  # Euclidean distance
        #return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    def reconstruct_path(came_from, current):
        path = deque()
        while current in came_from:  # while current is a key in the came_from dictionary
            path.appendleft(current)
            current = came_from[current]
        return path
    
    # Function which returns the cell to be explored depending on the current cell and on its parent cell 
    # This function is needed in order to determine the orientation of the robot and generate a path compatible with its movement
    def select_cell(current_,parent,yaw):
        current = (int(current_[0]), int(current_[1]))
        # The start doesn't have a parent: make a specific case for it
        if parent is not None:
            # Make the difference of the two coordinates
            lat = current[0]-parent[0]   # difference of latitude (difference of the row index)
            long = current[1]-parent[1]   # difference of longitude(difference of th column index)
        else:
            long = math.sin(yaw)
            lat = math.cos(yaw)
           
        # Now select te cell depending on the orientation
        if lat>0 and long>0:
            cell_to_be_explored=[(current[0]+1, current[1]), (current[0]+1, current[1]+1), (current[0], current[1]+1)]          
        elif lat>0 and long==0:
            cell_to_be_explored=[(current[0]+1, current[1]-1), (current[0]+1, current[1]), (current[0]+1, current[1]+1)]      
        elif lat>0 and long<0:  
            cell_to_be_explored=[(current[0], current[1]-1), (current[0]+1, current[1]-1), (current[0]+1, current[1])] 
        elif lat==0 and long<0:
            cell_to_be_explored=[(current[0]+1, current[1]-1), (current[0], current[1]-1), (current[0]-1, current[1]-1)]
        elif lat<0 and long<0:
            cell_to_be_explored=[(current[0], current[1]-1), (current[0]-1, current[1]-1), (current[0]-1, current[1])]
        elif lat<0 and long==0:
            cell_to_be_explored=[(current[0]-1, current[1]-1), (current[0]-1, current[1]), (current[0]-1, current[1]+1)]
        elif lat<0 and long>0: 
            cell_to_be_explored=[(current[0]-1, current[1]), (current[0]-1, current[1]+1), (current[0], current[1]+1)]
        elif lat==0 and long>0:   
            cell_to_be_explored=[(current[0]+1, current[1]+1), (current[0], current[1]+1), (current[0]-1, current[1]+1)]
        else:
            print("Error, check if the coordinates are in the range of possible values")  

       
        # If at the start Zeno faces a wall of obstacles, explore cells in all directions  
        if parent is None:
            count = 0
            for cell in cell_to_be_explored:
                if grid[cell[0]][cell[1]]:
                    count = count + 1 
                if count==3:
                    cell_final = [(current[0]+1, current[1]+1), (current[0]+1, current[1]), (current[0]+1, current[1]-1), (current[0], current[1]+1), (current[0], current[1]-1), (current[0]-1, current[1]+1), (current[0]-1, current[1]), (current[0]-1, current[1]-1)]
                else:
                    cell_final = cell_to_be_explored
        
        else:
            cell_final = cell_to_be_explored
            
        return cell_final       
          
        
    
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}   # creates an empty dictionary {node_i: came_from_i, ...} 
    cost_to_come = {start: 0}
    total_cost = {start: heuristic(start, goal)}
    step = 0
    while open_set:
        current = heapq.heappop(open_set)[1]  # pulls out the cell with the smallest f_score
        step+=1
        if current == goal:
            return reconstruct_path(came_from, current), step
        parent = came_from.get(current)
        cell_to_be_explored = select_cell(current,parent,yaw)
        #print("Cells to be explored are:", cell_to_be_explored)
        for neighbor in cell_to_be_explored:   
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and not grid[neighbor[0]][neighbor[1]]: 
                if neighbor[0]!=current[0] and neighbor[1]!=current[1]:   # diagonal movement
                    tentative_cost_to_come = cost_to_come[current] + math.sqrt(2)
                else:    # vertical or horizontal movement
                    tentative_cost_to_come = cost_to_come[current] + 1
                    
                for i in range (-1,2):
                    for j in range (-1,2):
                        if (0 <= neighbor[0]+i < grid.shape[0]) and (0 <= neighbor[1]+j < grid.shape[1]) and (grid[neighbor[0]+i][neighbor[1]+j]) == 1:
                            tentative_cost_to_come = cost_to_come[current] + 6
 
                # Now check if there's already a cost associated with this cell, if there's and is bigger than the previous one update it
                if tentative_cost_to_come < cost_to_come.get(neighbor, float("inf")):  # if the key neighbor is not found in the dictionary, returns float("inf")
                    came_from[neighbor] = current
                    cost_to_come[neighbor] = tentative_cost_to_come
                    total_cost[neighbor] = tentative_cost_to_come + heuristic(neighbor, goal)  # total cost 
                    heapq.heappush(open_set, (total_cost[neighbor], neighbor))   # adds the cell to the open set, maintaining the heap invariant               
    return None, None
