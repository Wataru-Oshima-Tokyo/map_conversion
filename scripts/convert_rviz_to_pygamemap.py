#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
In this code, we first subscribe to the /map topic to receive 
the OccupancyGrid messages that represent the map data in Rviz. 
Then, we convert the OccupancyGrid message to a 2D numpy array 
and process the data to convert it to Pygame coordinates. 
Finally, we initialize Pygame, draw the map on the Pygame s
creen and run the main loop to display the map.
"""





import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pygame
from geometry_msgs.msg import PoseWithCovarianceStamped


class MAP_CONVERSION:
    def __init__(self):
        rospy.init_node('map_conversion')
        self.pygame_map = None
        self.screen = None
        self.initial = True
        self.rate = rospy.Rate(10)
        self.surface = None
        self.show_map = False
        self.origin = [0,0]
        self.windowsize = [0,0]
        # Subscribing to the map topic
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)


    def map_callback(self, map_msg):
        coefficient = 6
        if self.initial:
            self.windowsize[0] = map_msg.info.width
            self.windowsize[1] = map_msg.info.height
            self.origin[0] = self.windowsize[0]*coefficient/2
            self.origin[1] = self.windowsize[1]*coefficient/2
            self.screen = pygame.display.set_mode((self.windowsize[0]* coefficient, self.windowsize[1]* coefficient))
            self.initial = False
        print(map_msg.info.width,map_msg.info.height)
        print(map_msg.info.origin)
        self.screen.fill((255, 255, 255))
        map_data = np.array(map_msg.data, dtype=np.int8)
        map_data = map_data.reshape((self.windowsize[0], self.windowsize[1]))

        # Convert the map data to Pygame coordinates
        self.pygame_map = np.zeros((self.windowsize[1], self.windowsize[0], 3), dtype=np.uint8)
        for i in range(self.windowsize[1]):
            for j in range(self.windowsize[0]):
                value = map_data[i][j]
                if value == 0:
                    # Free space
                    self.pygame_map[i][j] = [255, 255, 255]
                elif value == 100:
                    # Occupied space
                    self.pygame_map[i][j] = [0, 0, 0]
                else:
                    # Unknown space
                    self.pygame_map[i][j] = [128, 128, 128]
        self.surface = pygame.surfarray.make_surface(self.pygame_map)
        self.surface = pygame.transform.scale(self.surface, (self.windowsize[0] * coefficient, self.windowsize[1]*coefficient))
        self.show_map = True
        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()
        # pygame.display.update()

    def amcl_pose_callback(self, msg):
        # Extract the current pose from the message
        pose = msg.pose.pose

        # # Initialize a listener for transformations
        # listener = tf.TransformListener()

        # # Wait for the transformation to become available
        # listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

        # # Transform the pose from the AMCL_pose coordinate frame to the map frame
        # (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
        # map_pose = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
        #                                                     tf.transformations.quaternion_matrix(rot))
        # transformed_pose = tf.transformations.concatenate_matrices(map_pose,
        #                                                             tf.transformations.translation_matrix((pose.position.x,
        #                                                                                                   pose.position.y,
        #                                                                                                   pose.position.z)))

        # Extract the transformed x, y and z coordinates
        x = pose.position.x
        y = pose.position.y
        map_x = self.origin[0] - x * self.windowsize[0] #165 is calculated by 
        #(window_size/maxGrid_size) * (maxgrid/acutally used grid) = (1000/20)*(20/6)
        # which is also equal to window_size / actually used grid = 1000/6 = 166.67
        map_y = self.origin[1] - y * self.windowsize[1]
        # Clear the screen
        if self.show_map:
            self.screen.fill((255, 255, 255))
            self.screen.blit(self.surface, (0, 0))
            # Draw a circle at the transformed x and y coordinates
            pygame.draw.circle(self.screen, (0, 0, 255), (int(map_y), int(map_x)), 10)

            # Update the display
            


if __name__ == "__main__":
    # Initialize Pygame

    pygame.init()
    
    map_conversion = MAP_CONVERSION()
    # while not rospy.is_shutdown():
        # Draw the map on the Pygame screen
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Quit")
        if map_conversion.show_map:
            pygame.display.update()

        map_conversion.rate.sleep()
    # rospy.spin()
    
    # Quit Pygame
    pygame.quit()