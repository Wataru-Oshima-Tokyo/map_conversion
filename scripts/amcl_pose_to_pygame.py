#!/usr/bin/python
# -*- coding: utf-8 -*-

# Import required libraries
import rospy
import tf
import pygame
from geometry_msgs.msg import PoseWithCovarianceStamped

# Initialize Pygame window
pygame.init()
window_size = (1000, 1000)
origin = (window_size[0]/2,window_size[1]/2)
screen = pygame.display.set_mode(window_size)

# Initialize node
rospy.init_node('visualize_amcl_pose_in_pygame')

# Create a subscriber to listen to the AMCL_pose topic
def amcl_pose_callback(msg):
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
    map_x = origin[0] - x * 165 #165 is calculated by 
    #(window_size/maxGrid_size) * (maxgrid/acutally used grid) = (1000/20)*(20/6)
    # which is also equal to window_size / actually used grid = 1000/6 = 166.67
    map_y = origin[1] - y * 165
    # Clear the screen
    screen.fill((255, 255, 255))

    # Draw a circle at the transformed x and y coordinates
    pygame.draw.circle(screen, (0, 0, 255), (int(map_y), int(map_x)), 10)

    # Update the display
    pygame.display.update()

# Subscribe to the AMCL_pose topic
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

# Spin to keep the node running
while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            rospy.signal_shutdown("Quit")

# Quit Pygame
pygame.quit()