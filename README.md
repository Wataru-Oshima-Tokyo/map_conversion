# Map Conversion
This code converts the OccupancyGrid messages, which represent the map data in Rviz, to a 2D numpy array and processes the data to convert it to Pygame coordinates. The code then initializes Pygame, draws the map on the Pygame screen, and runs the main loop to display the map.

## Requirements
Python 2.7 or above
ROS (Robot Operating System)
Pygame
Numpy
Usage

### Run the ROS master node by using the following command:
Copy code
'''
roscore
'''


### Run a map_server node that will publish the map data. For example:
Copy code
'''
rosrun map_server map_server {path_to_map}
'''
### Run the .py script to start the map conversion:
Copy code
'''
rosrun map_conversion 
'''
Components
The code consists of two ROS subscribers:

Subscriber to the "/map" topic to receive the OccupancyGrid messages that represent the map data in Rviz.
Subscriber to the "/amcl_pose" topic to receive the current pose of the robot.
The code also includes two callback functions:

map_callback function: This function is called whenever a new message is received on the "/map" topic. The function converts the map data from the OccupancyGrid message to a 2D numpy array, processes the data to convert it to Pygame coordinates, and draws the map on the Pygame screen.

amcl_pose_callback function: This function is called whenever a new message is received on the "/amcl_pose" topic. The function extracts the current pose of the robot from the message and transforms the pose from the AMCL_pose coordinate frame to the map frame.

Note
The current implementation only converts the map data and displays it on the Pygame screen, but the code can be further extended to add additional functionality, such as display of the robot's current position on the map.
