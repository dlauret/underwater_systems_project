OBSTACLE AVOIDANCE PROJECT "README":

Download "obstacle_avoidance" and "obstacle_avoidance_bag" directories inside the zip folder.
NOTE: once the "obstacle_avoidance" package has been compiled through the  "catkin_make" command, be sure to make executable all the python scripts inside the src directory

----------------------------------------------------------------------------------------------------------------------------------------------
Instructions for experiments' bags reading:
1) Run the launch file:
   roslaunch obstacle_avoidance visualization_nodes.launch
2) From the "obstacle_avoidance_bag" directory run the rosbag play command:
   rosbag play obstacle_avoidance_bag_hard.bag
OR
   rosbag play obstacle_avoidance_bag_soft.bag

----------------------------------------------------------------------------------------------------------------------------------------------
Instructions for a generic experiment which include the reading of a map, the connection to Zeno (or its simulator) or the usage of our dynamic simulator for Zeno:
1) Write a .txt file with the correct specification of map and obstacle's coordinates and rename it as "map_file.txt"; then put it inside the src directory.
   NOTE: the map_file.txt already inside the src directory refers to the hard experiment scenario

At this point choose between one of the two options below:

a) Use of the dynamic Zeno simulator
 a.1) Make sure that the starting position of Zeno (inside the "zeno_dynamic_model.py" script) and of the goal (in the launch file) are
 both feasible, i.e. within the map limits
 a.2) Run the launch file in this order:

     roslaunch obstacle_avoidance visualization_nodes.launch
     roslaunch obstacle_avoidance main_nodes.launch

 OR you can run all the nodes together with the following command:
     roslaunch obstacle_avoidance all_nodes.launch

 a.3) In order to move Zeno, run the following command:
     rosparam set /MODE_ZENO 1 
 a.4) To stop Zeno, run:
     rosparam set /MODE_ZENO stop

b) Connection to Zeno
 b.1) Comment the Zeno simulator node inside the launch files 
 b.2) Repeat a.2)

NOTE: The rosbag node is commented by default inside the launch files: if in need to record uncomment it and modify the absolute path correctly based on your preference

----------------------------------------------------------------------------------------------------------------------------------------------


