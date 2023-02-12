<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
  </head>
  <body>
    <h1>Map Conversion</h1>
    <p>
      This code converts the OccupancyGrid messages, which represent the map data in Rviz, to a 2D numpy array and processes the data to convert it to Pygame coordinates. The code then initializes Pygame, draws the map on the Pygame screen, and runs the main loop to display the map.
    </p>
    <h2>Requirements</h2>
    <ul>
      <li>Python 2.7 or above</li>
      <li>ROS (Robot Operating System)</li>
      <li>Pygame</li>
      <li>Numpy</li>
    </ul>
    <h2>Usage</h2>
    <ol>
      <li>Run the ROS master node by using the following command:<br>
        <code>roscore</code>
      </li>
      <li>Launch a ROS node that will publish the map data. For example:<br>
        <code>rosrun map_server map_server {path_to_map}</code>
      </li>
      <li>Run the convert_amcl_pose_to_map_coordinate.pyscript to start the map conversion:<br>
        <code>rosrun  map_conversion convert_amcl_pose_to_map_coordinate.py</code>
      </li>
    </ol>
    <h2>Components</h2>
    <p>
      The code consists of two ROS subscribers:
    </p>
    <ul>
      <li>Subscriber to the "/map" topic to receive the OccupancyGrid messages that represent the map data in Rviz.</li>
      <li>Subscriber to the "/amcl_pose" topic to receive the current pose of the robot.</li>
    </ul>
    <p>
      The code also includes two callback functions:
    </p>
    <ul>
      <li>map_callback function: This function is called whenever a new message is received on the "/map" topic. The function converts the map data from the OccupancyGrid message to a 2D numpy array, processes the data to convert it to Pygame coordinates, and draws the map on the Pygame screen.</li>
      <li>amcl_pose_callback function: This function is called whenever a new message is received on the "/amcl_pose" topic. The function extracts the current pose of the robot from the message and transforms the pose from the AMCL_pose coordinate frame to the map frame.</li>
    </ul>
    <p>
      <strong>Note:</strong> The current implementation only converts the map data and displays it on the Pygame screen, but the code can be further extended to add additional functionality, such as display of the robot's current position on the map.
    </p>
  </body>
</html>
