#! /usr/bin/env python

import sys
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import map_model
import explorer


class MovementNode:

    def __init__(self):
        """
        Setup the MovementNode:
         - Initialises the state
         - Gathers the maps and creates their models
         - Creates all publishers and subscribers
        """

        self.pose = Pose()  # The robots believed pose

        # Wait for the maps to be broadcast
        rospy.loginfo("Waiting for a map...")
        try:
            occupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
            available_space = rospy.wait_for_message("/available_space", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting maps. Check that you have a map_server"
                         " running: rosrun map_server map_server <mapname> ")
            sys.exit(1)
        rospy.loginfo("Maps received. %d X %d, %f px/m." %
                      (occupancy_map.info.width, occupancy_map.info.height,
                       occupancy_map.info.resolution))

        # Create the map models from the received maps
        self.occ_map_model = map_model.MapModel(occupancy_map)
        self.avail_space_model = map_model.MapModel(available_space)

        self.explorer = explorer.Explorer(self.avail_space_model)

        # Listen for estimated poses from amcl
        self.estimatedPoseListener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                                                      self.estimated_pose_listener, queue_size=100)


        rospy.loginfo("Init complete!")

        # Wait for the initial pose and trigger when it's received
        rospy.loginfo("Waiting for initial pose...")
        try:
            pose_message = rospy.wait_for_message("/initialpose", PoseWithCovarianceStamped)
        except:
            rospy.logerr("Failed to receive initial pose!")
        rospy.loginfo("Received initial pose!")
        self.pose = pose_message

    def estimated_pose_listener(self, pose_message):
        """
        Update the believed pose of the robot when amcl sends an update
        :param pose_message: THe believed pose of the robot according to amcl
        """
        self.pose = pose_message
        self.explorer.update_map(self.pose.pose.pose)

    def update_visual_space_plot(self):
        plt.draw(self.visual_space)


if __name__ == '__main__':
    rospy.init_node(name="joy_movement_node", log_level=rospy.INFO)
    rospy.loginfo("Starting joystick movement_node...")
    mv = MovementNode()
    rospy.loginfo("Entering spin...")
    rospy.spin()
    # mv.start()
