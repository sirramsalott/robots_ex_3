from enum import Enum

import rospy
import actionlib
from robots_ex_3.msg import MoveBaseAction, MoveBaseGoal
from msg import TrackFace, FaceLocked
from msgs.msg import Empty


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
        self.base_state = BaseStates.EXPLORE
        self.explore_state = ExploreStates.NO_GOAL

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
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.next_waypoint()
        self.current_goal = goal
        self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

    def track_face(self):
        """
        Move according to the most recent face message
        :return:
        """
        # TODO Get x center of image
        # TODO Get size of image
        # TODO move towards the face

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
        :return:
        """


if __name__ == '__main__':
    mv = MovementNode()
    mv.start()
