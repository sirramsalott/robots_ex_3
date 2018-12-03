#! /usr/bin/env python

from enum import Enum
import sys
import rospy
import actionlib
import face_detection_lib as fd
import movement_model as mm
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from robots_exercise_3.msg import TrackFace, StudentFaceLocked, NewFaceLocked
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped, Twist
import map_model
import math
import util
import random as rand
import matplotlib.pyplot as plt
import re
import numpy as np
import explorer


class BaseStates(Enum):
    EXPLORE = 1
    TRACK_FACE = 2
    TRACK_LOSS = 3
    STILL = 4
    BOOT = 5


class ExploreStates(Enum):
    MOVE_TO_GOAL = 1
    AT_GOAL = 2
    NO_GOAL = 3
    GOAL_REJECTED = 4


class ExploreResponses(Enum):
    GOAL_IN_PROGRESS = 1


class MovementNode:

    def __init__(self):
        """
        Setup the MovementNode:
         - Initialises the state
         - Gathers the maps and creates their models
         - Creates all publishers and subscribers
        """

        # Initialise the movement state
        self.base_state = BaseStates.BOOT
        self.explore_state = ExploreStates.NO_GOAL

        self.face_threshold = 10  # how many pixels either side of the centre are classed as central

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

        # Subscribe to the facial topics
        self.faceTrackListener = rospy.Subscriber("/track_face", TrackFace, self.face_listener, queue_size=1)
        self.faceLockListener = rospy.Subscriber("/face_locked", StudentFaceLocked, self.face_lock_listener, queue_size=1)
        self.faceLossListener = rospy.Subscriber("/face_lost", Empty, self.face_loss_listener, queue_size=1)
        self.facePendListener = rospy.Subscriber("/face_pend", Empty, self.face_pend_listener, queue_size=1)

        # Listen for estimated poses from amcl
        self.estimatedPoseListener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                                                      self.estimated_pose_listener, queue_size=1)

        # Publish the robots movement commands
        self.movePublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # The nodes reaction publisher and subscriber
        self.reactListener = rospy.Subscriber("/movement_react", Empty, self.react, queue_size=1)
        self.reactPublisher = rospy.Publisher("/movement_react", Empty, queue_size=1)

        # The action client for move_base
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.recent_face = None
        self.current_goal = None
        self.goal_handler = None

        rospy.loginfo("Init complete!")

        # Wait for the initial pose and trigger when it's received
        rospy.loginfo("Waiting for initial pose...")
        try:
            pose_message = rospy.wait_for_message("/initialpose", PoseWithCovarianceStamped)
        except:
            rospy.logerr("Failed to receive initial pose!")
        rospy.loginfo("Received initial pose!")
        self.pose = pose_message
        self.base_state = BaseStates.EXPLORE
        self.trigger()

    def react(self, msg):
        """
        Process a reaction to being triggered according to the current state of the node
        :param msg: Unused
        """
        del msg
        rospy.loginfo("Reacting...")
        self.print_state()
        if self.base_state == BaseStates.EXPLORE:
            self.explore()
            return

        if self.explore_state != ExploreStates.NO_GOAL:
            self.explore_state = ExploreStates.NO_GOAL
            rospy.loginfo("Cancelling current goal...")
            self.move_client.cancel_all_goals()
            rospy.loginfo("Cancelled!")

        if self.base_state == BaseStates.TRACK_FACE:
            self.track_face(True)
            return

        if self.base_state == BaseStates.TRACK_LOSS:
            self.track_face(False)
            return

        if self.base_state == BaseStates.STILL:
            self.track_face(False)

    def print_state(self):
        """
        PPrint the current state of the node
        """
        rospy.loginfo("Base: {}, Explore State: {}".format(self.base_state.name, self.explore_state.name))

    def trigger(self):
        """
        Trigger the robot to cause an async reaction
        """
        rospy.loginfo("Triggering...")
        self.reactPublisher.publish(Empty())

    def estimated_pose_listener(self, pose_message):
        """
        Update the believed pose of the robot when amcl sends an update
        :param pose_message: THe believed pose of the robot according to amcl
        """
        self.pose = pose_message

    def active_cb(self):
        """
        Log the goal being acted upon
        """
        p = self.current_goal.target_pose.pose.position
        rospy.loginfo(
            "Goal pose ({}, {}) is now being processed by the Action Server...".format(p.x, p.y))

    def feedback_cb(self, feedback):
        """
        Log that feedback has been received
        :param feedback: The received feedback message
        """
        del feedback
        p = self.current_goal.target_pose.pose.position
        rospy.loginfo("Feedback for goal ({}, {})".format(p.x, p.y))

    def done_cb(self, status, result):
        """
        Log the completed goal
        Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        :param status: The status of the goal
        :param result: The result of execution (unused)
        """
        del result
        p = self.current_goal.target_pose.pose.position
        # Goal cancelled
        if status == 2:
            rospy.loginfo(
                "Goal pose ({}, {}) received a cancel request after it started executing, completed execution!".format(
                    p.x, p.y))
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal pose ({}, {}) reached".format(p.x, p.y))
            self.explore_state = ExploreStates.AT_GOAL
            self.trigger()
            return

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal pose ({}, {}) was aborted by the Action Server".format(p.x, p.y))
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal pose ({}, {}) has been rejected by the Action Server".format(p.x, p.y))
            self.explore_state = ExploreStates.GOAL_REJECTED
            self.trigger()
            return

        # Goal cancelled before execution started
        if status == 8:
            rospy.loginfo(
                "Goal pose ({}, {}) received a cancel request before it started executing, successfully cancelled!".format(
                    p.x, p.y))
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()

    def explore(self):
        """
        Explore the environment based on the current explore state
        """
        # Already exploring?
        if self.explore_state == ExploreStates.MOVE_TO_GOAL:
            return ExploreResponses.GOAL_IN_PROGRESS

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.explorer.next_waypoint()
        self.goal_handler = self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        self.current_goal = goal
        self.explore_state = ExploreStates.MOVE_TO_GOAL

    def track_face(self, forward):
        """
        Move according to the most recent face message
        """
        rospy.loginfo("track_face executing...")
        face = self.recent_face
        if face == None:
           rospy.logwarn("No recent face: Aborting!")
           return
        x_centre = (face.left + face.right) / 2
        img_x_centre = fd.image_width / 2
        rospy.loginfo("Face centre: {}, img centre: {}".format(x_centre, img_x_centre))

        move = Twist()
        if forward:
            move.linear.x = mm.TRACK_FACE_FORWARD
        else:
            move.linear.x = 0
        if x_centre < img_x_centre - self.face_threshold:
            move.angular.z = mm.TRACK_FACE_LEFT
        elif x_centre > img_x_centre + self.face_threshold:
            move.angular.z = mm.TRACK_FACE_RIGHT
        rospy.loginfo("Moving x:{} z:{} ...".format(move.linear.x, move.angular.z))
        self.movePublisher.publish(move)

    def face_listener(self, face):
        """
        Move according to the received facial data
        """
        rospy.loginfo("Track face received!")
        self.recent_face = face
        self.base_state = BaseStates.TRACK_FACE
        self.trigger()

    def face_lock_listener(self, face):
        """
        Stand still when the face is locked
        """
        rospy.loginfo("Face lock received!")
        self.recent_face = face
        self.base_state = BaseStates.STILL
        self.trigger()

    def face_pend_listener(self, msg):
        """
        Keep tracking but don't move forward
        """
        rospy.loginfo("Face pend received!")
        self.base_state = BaseStates.TRACK_LOSS
        self.trigger()

    def face_loss_listener(self, msg):
        """
        Return to exploration when the face is lost
        """
        rospy.loginfo("Face loss received!")
        self.base_state = BaseStates.EXPLORE
        self.trigger()

    def update_visual_space_plot(self):
        plt.draw(self.visual_space)


if __name__ == '__main__':
    rospy.init_node(name="movement_node", log_level=rospy.INFO)
    rospy.loginfo("Starting movement_node...")
    mv = MovementNode()
    rospy.loginfo("Entering spin...")
    rospy.spin()
    # mv.start()
