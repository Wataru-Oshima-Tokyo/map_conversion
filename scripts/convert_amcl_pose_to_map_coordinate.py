#!/usr/bin/python
# -*- coding: utf-8 -*-

# Import required libraries
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

# Initialize node
rospy.init_node('transform_amcl_pose_to_map')

# Create a subscriber to listen to the AMCL_pose topic
def amcl_pose_callback(msg):
    # Extract the current pose from the message
    pose = msg.pose.pose

    # Initialize a listener for transformations
    listener = tf.TransformListener()

    # Wait for the transformation to become available
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

    # Transform the pose from the AMCL_pose coordinate frame to the map frame
    (trans, rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
    map_pose = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
                                                        tf.transformations.quaternion_matrix(rot))
    transformed_pose = tf.transformations.concatenate_matrices(map_pose,
                                                                tf.transformations.translation_matrix((pose.position.x,
                                                                                                      pose.position.y,
                                                                                                      pose.position.z)))

    # Extract the transformed x, y and z coordinates
    x = transformed_pose[0][3]
    y = transformed_pose[1][3]
    z = transformed_pose[2][3]

    # Print the transformed x, y and z coordinates
    print("Transformed x: ", x)
    print("Transformed y: ", y)
    print("Transformed z: ", z)

# Subscribe to the AMCL_pose topic
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

# Spin to keep the node running
rospy.spin()