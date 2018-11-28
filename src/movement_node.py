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
import sensor_model
import math
from util import rotateQuaternion
import random as rand
import matplotlib.pyplot as plt
import re
import numpy as np


class BaseStates(Enum):
    EXPLORE = 1
    TRACK_FACE = 2
    STILL = 3
    BOOT = 4 


class ExploreStates(Enum):
    MOVE_TO_GOAL = 1
    AT_GOAL = 2
    NO_GOAL = 3
    GOAL_REJECTED = 4


class ExploreResponses(Enum):
    GOAL_IN_PROGRESS = 1


class MovementNode:

    def __init__(self):

        self.sensor_model = sensor_model.SensorModel()

        self.base_state = BaseStates.BOOT
        self.explore_state = ExploreStates.NO_GOAL

        self.face_threshold = 10  # how many pixels either side of the centre are classed as central

        self.pose = Pose()

        # Sort out the map
        self.occupancy_map = OccupancyGrid()
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
        self.set_map(occupancy_map, available_space)

        #available_space_file = rospy.get_param("~available_space_file")
        #self.available_space = self.load_available_space(available_space_file)
        #self.visual_space = self.load_available_space(available_space_file)

        # Subscribe to the facial topics
        self.faceTrackListener = rospy.Subscriber("/track_face", TrackFace, self.faceListener, queue_size=1)
        self.faceLockListener = rospy.Subscriber("/face_locked", StudentFaceLocked, self.faceLockListener, queue_size=1)
        self.faceLossListener = rospy.Subscriber("/face_lost", Empty, self.faceLossListener, queue_size=1)

        self.initialposeListener = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialposeListener, queue_size=1)
        self.estimatedposeListener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.estimated_pose_listener, queue_size=1)

        self.movePublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.reactListener = rospy.Subscriber("/movement_react", Empty, self.react, queue_size=1)
        self.reactPublisher = rospy.Publisher("/movement_react", Empty, queue_size=1)

        # TODO publishers

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.recentFace = None
        self.current_goal = None
        self.goal_handler = None

        #plt.ion()
        #plt.show()
        #plt.draw(self.visual_space)

	rospy.loginfo("Init complete!")

    def react(self, msg):
        """
        Process a single action
        :return:
        """
        rospy.loginfo("Reacting...")
        self.print_state()
        if self.base_state == BaseStates.EXPLORE:
            self.explore()
            return

        if self.explore_state != ExploreStates.NO_GOAL:
            self.explore_state = ExploreStates.NO_GOAL
            self.goal_handler.cancel()

        if self.base_state == BaseStates.TRACK_FACE:
            self.track_face()
            return
        if self.base_state == BaseStates.STILL:
            # TODO leave to callback
            return

    def print_state(self):
	rospy.loginfo("Base: {}, Explore State: {}".format(self.base_state.name, self.explore_state.name))

    def trigger(self):
        rospy.loginfo("Triggering...")
        self.reactPublisher.publish(Empty())

    def initialposeListener(self, poseMessage):
	rospy.loginfo("Received initial pose!")
        self.pose = poseMessage
	self.base_state = BaseStates.EXPLORE
        self.trigger()

    def estimated_pose_listener(self, poseMessage):
        #(x,y) = self.pose_to_map_coords(self.pose)
        #self.visual_space[x, y] = 0
        self.pose = poseMessage
        #(x,y) = self.pose_to_map_coords(self.pose)
        #self.visual_space[x, y] = 100
        #self.update_visual_space_plot()	
	
    def active_cb(self):
        rospy.loginfo(
            "Goal pose " + str(self.current_goal.target_pose.pose.position) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        rospy.loginfo("Feedback for goal " + str(self.current_goal.target_pose.pose.position) + ": " + str(feedback))

    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.current_goal.target_pose.pose.position) + " received a cancel request after it started executing, completed execution!")
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.current_goal.target_pose.pose.position) + " reached")
            self.explore_state = ExploreStates.AT_GOAL
            self.trigger()
            return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.current_goal.target_pose.pose.position) + " was aborted by the Action Server")
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.current_goal.target_pose.pose.position) + " has been rejected by the Action Server")
            self.explore_state = ExploreStates.GOAL_REJECTED
            self.trigger()
            return

        if status == 8:
            rospy.loginfo("Goal pose " + str(
                self.current_goal.target_pose.pose.position) + " received a cancel request before it started executing, successfully cancelled!")
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()

    def explore(self):
        """
        Explore the environment based on the current explore state
        :return:
        """
        # Already exploring?
        if self.explore_state == ExploreStates.MOVE_TO_GOAL:
            return ExploreResponses.GOAL_IN_PROGRESS

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.next_waypoint()
        self.current_goal = goal
        self.goal_handler = self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def track_face(self):
        """
        Move according to the most recent face message
        :return:
        """
        rospy.loginfo("face_track executing...")
        face = self.recentFace
        x_centre = face.left + face.right / 2
        img_x_centre = fd.image_width / 2

	move = Twist()
        move.linear.x = mm.TRACK_FACE_FORWARD
        if x_centre < img_x_centre - self.face_threshold:
            move.angular.z = mm.TRACK_FACE_LEFT
        elif x_centre > img_x_centre + self.face_threshold:
            move.angular.z = mm.TRACK_FACE_RIGHT
	rospy.loginfo("Moving x:{} z:{} ...".format(move.linear.x, move.angular.z))
        self.movePublisher.publish(move)

    def faceListener(self, face):
        """
        Move according to the received facial data
        :return:
        """
	rospy.loginfo("Track face received!")
        self.recentFace = face
        if self.base_state == BaseStates.EXPLORE:
            self.base_state = BaseStates.TRACK_FACE
            self.trigger()
            return
        self.track_face()

    def faceLockListener(self, face):
        """
        Stand still when the face is locked
        :return:
        """
	rospy.loginfo("Face lock received!")
        self.base_state = BaseStates.STILL

    def faceLossListener(self, msg):
        """
        Return to exploration when the face is lost
        :return:
        """
	rospy.loginfo("Face loss received!")
        self.base_state = BaseStates.EXPLORE
        self.trigger()

    def next_waypoint(self):
        """
        Determine the next waypoint to navigate to
        :return: Random point on the map
        """
        while True:
            x,y = self.map_coords_to_world(rand.uniform(0, self.occupancy_map.info.width - 1),
                                           rand.uniform(0, self.occupancy_map.info.height - 1))
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y

            if not self.map_pose_occupied(new_pose, self.available_space):
                rand_a = rand.uniform(0, 2 * math.pi)
                new_pose.orientation = rotateQuaternion(q_orig=Quaternion(0, 0, 0, 1),
                                                        yaw=rand_a)

		rospy.loginfo("x: {}, y: {}".format(x, y))
                return new_pose

    def set_map(self, occupancy_map, available_space):
        """ Set the map for localisation """
        self.occupancy_map = occupancy_map
        self.available_space = available_space
        self.sensor_model.set_map(occupancy_map)

    def pose_to_map_coords(self, pose):
        ox = pose.position.x
        oy = pose.position.y

        map_x = math.floor((
                                   ox - self.sensor_model.map_origin_x) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_width / 2
        map_y = math.floor((
                                   oy - self.sensor_model.map_origin_y) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_height / 2

        return int(math.floor(map_x)), int(math.floor(map_y))

    def map_coords_to_world(self, map_x, map_y):
        x = self.sensor_model.map_resolution * (
                map_x - self.sensor_model.map_width / 2) + self.sensor_model.map_origin_x
        y = self.sensor_model.map_resolution * (
                map_y - self.sensor_model.map_height / 2) + self.sensor_model.map_origin_y

        return x, y

    def map_pose_occupied(self, pose, map_space):
        x, y = self.pose_to_map_coords(pose)
        threshold = 80
        prob_occupied = map_space.data[int(x + y * self.occupancy_map.info.width)]

        return prob_occupied == -1 or prob_occupied > threshold

#    def pose_space_available(self, pose):
#        x, y = self.pose_to_map_coords(pose)
#        return self.space_available_coords((x,y))
	
    def map_coords_occupied(self, coords, map_space):
        (x, y) = coords
        threshold = 80
        prob_occupied = map_space.data[int(x + y * self.occupancy_map.info.width)]

        return prob_occupied == -1 or prob_occupied > threshold

#    def space_available_coords(self, coords):
#        (x, y) = coords
#        return self.available_space[int(y), int(x)] != 0

#    def load_available_space(self, available_space_file):
#        
#        def read_pgm(filename, byteorder='>'):
#
#            with open(filename, 'rb') as f:
#                buffer = f.read()
#            try:
#                header, width, height, maxval = re.search(
#                    b"(^P5\s(?:\s*#.*[\r\n])*"
#                    b"(\d+)\s(?:\s*#.*[\r\n])*"
#                    b"(\d+)\s(?:\s*#.*[\r\n])*"
#                    b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
#            except AttributeError:
#                raise ValueError("Not a raw PGM file: '%s'" % filename)
#            return np.frombuffer(buffer,
#                                    dtype='u1' if int(maxval) < 256 else byteorder+'u2',
#                                    count=int(width)*int(height),
#                                    offset=len(header)
#                                    ).reshape((int(height), int(width)))
#        return read_pgm(available_space_file)

    def update_visual_space_plot(self):
        plt.draw(self.visual_space)

if __name__ == '__main__':
    rospy.init_node(name="face_detect", log_level=rospy.INFO)
    rospy.loginfo("Starting movement_node...")
    mv = MovementNode() 
    rospy.loginfo("Entering spin...")
    rospy.spin()
    #mv.start()
