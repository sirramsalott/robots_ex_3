from enum import Enum

import rospy
import actionlib
import face_detection_lib as fd
import movement_model as mm
from robots_ex_3.msg import MoveBaseAction, MoveBaseGoal
from msg import TrackFace, FaceLocked, OccupancyGrid
from msgs.msg import Empty
from geometry_msgs.msg import Pose, Quaternion
import sensor_model
import math
from util import rotateQuaternion
import sys
import random as rand


class BaseStates(Enum):
    EXPLORE = 1
    TRACK_FACE = 2
    STILL = 3


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

        self.base_state = BaseStates.EXPLORE
        self.explore_state = ExploreStates.NO_GOAL

        self.face_threshold = 10  # how many pixels either side of the centre are classed as central

        # Sort out the map
        self.occupancy_map = OccupancyGrid()
        rospy.loginfo("Waiting for a map...")
        try:
            occupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                         " running: rosrun map_server map_server <mapname> ")
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (occupancy_map.info.width, occupancy_map.info.height,
                       occupancy_map.info.resolution))
        self.set_map(occupancy_map)

        # Subscribe to the facial topics
        self.faceTrackListener = rospy.Subscriber("/track_face", TrackFace, self.faceListener(), queue_size=1)
        self.faceLockListener = rospy.Subscriber("/face_locked", FaceLocked, self.faceListener(), queue_size=1)
        self.faceLossListener = rospy.Subscriber("/face_lost", Empty, self.faceListener(), queue_size=1)

        self.reactListener = rospy.Subscriber("/movement_react", Empty, self.react(), queue_size=1)
        self.reactPublisher = rospy.Publisher("/movement_react", Empty)

        # TODO publishers

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.recentFace = None
        self.current_goal = None

        self.trigger()

    def react(self):
        """
        Process a single action
        :return:
        """
        if self.base_state == BaseStates.EXPLORE:
            self.explore()
            return

        if self.explore_state != ExploreStates.NO_GOAL:
            self.explore_state = ExploreStates.NO_GOAL
            self.move_client.cancelGoal()

        if self.base_state == BaseStates.TRACK_FACE:
            # TODO leave to callback
            return
        if self.base_state == BaseStates.STILL:
            # TODO leave to callback
            return

    def trigger(self):
        self.reactPublisher.publish(Empty())

    def active_cb(self):
        rospy.loginfo(
            "Goal pose " + str(self.current_goal.pose.pose) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        rospy.loginfo("Feedback for goal " + str(self.current_goal.pose.pose) + ": " + str(feedback))

    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.current_goal.pose.pose) + " received a cancel request after it started executing, completed execution!")
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.current_goal.pose.pose) + " reached")
            self.explore_state = ExploreStates.AT_GOAL
            self.trigger()
            return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.current_goal.pose.pose) + " was aborted by the Action Server")
            self.explore_state = ExploreStates.NO_GOAL
            self.trigger()
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.current_goal.pose.pose) + " has been rejected by the Action Server")
            self.explore_state = ExploreStates.GOAL_REJECTED
            self.trigger()
            return

        if status == 8:
            rospy.loginfo("Goal pose " + str(
                self.current_goal.pose.pose) + " received a cancel request before it started executing, successfully cancelled!")
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
        self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        print("Spinning...")
        rospy.spin()
        print("Passed spin")

    def track_face(self):
        """
        Move according to the most recent face message
        :return:
        """

        face = self.recentFace
        x_centre = face.left + face.right / 2
        img_x_centre = fd.image_width / 2

        if x_centre < img_x_centre - self.face_threshold:
            pass  # TODO spin left
        elif x_centre > img_x_centre + self.face_threshold:
            pass  # TODO spin right

        # TODO move forward
        return

    def faceListener(self, face):
        """
        Move according to the received facial data
        :return:
        """

        if self.base_state == BaseStates.EXPLORE:
            self.base_state = BaseStates.TRACK_FACE
            self.trigger()

        self.recentFace = face
        self.track_face()

    def faceLockListener(self, face):
        """
        Stand still when the face is locked
        :return:
        """
        self.base_state = BaseStates.STILL

    def faceLossListener(self):
        """
        Return to exploration when the face is lost
        :return:
        """
        self.base_state = BaseStates.EXPLORE
        self.trigger()

    def next_waypoint(self):
        """
        Determine the next waypoint to navigate to
        :return: Random point on the map
        """
        while True:
            rand_x = rand.uniform(0, self.occupancy_map.info.width - 1)
            rand_y = rand.uniform(0, self.occupancy_map.info.height - 1)

            if not self.map_coords_occupied((rand_x, rand_y)):
                rand_a = rand.uniform(0, 2 * math.pi)
                new_pose = Pose()
                new_pose.orientation = rotateQuaternion(q_orig=Quaternion(0, 0, 0, 1),
                                                        yaw=rand_a)

                world_x, world_y = self.map_coords_to_pose(rand_x, rand_y)

                new_pose.location.x = world_x
                new_pose.location.y = world_y

                return new_pose

    def set_map(self, occupancy_map):
        """ Set the map for localisation """
        self.occupancy_map = occupancy_map
        self.sensor_model.set_map(occupancy_map)

    def pose_to_map_coords(self, pose):
        ox = pose.position.x
        oy = pose.position.y

        map_x = math.floor((
                                   ox - self.sensor_model.map_origin_x) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_width / 2
        map_y = math.floor((
                                   oy - self.sensor_model.map_origin_y) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_height / 2

        return int(math.floor(map_x)), int(math.floor(map_y))

    def map_coords_to_pose(self, map_x, map_y):
        x = self.sensor_model.map_resolution * (
                map_x - self.sensor_model.map_width / 2) + self.sensor_model.map_origin_x
        y = self.sensor_model.map_resolution * (
                map_y - self.sensor_model.map_height / 2) + self.sensor_model.map_origin_y

        return x, y

    def map_pose_occupied(self, pose):
        x, y = self.pose_to_map_coords(pose)
        return self.map_coords_occupied((x, y))

    def map_coords_occupied(self, coords):
        (x, y) = coords
        threshold = 80
        prob_occupied = self.occupancy_map.data[x + y * self.occupancy_map.info.width]

        return prob_occupied == -1 or prob_occupied > threshold


if __name__ == '__main__':
    mv = MovementNode()
    mv.start()
