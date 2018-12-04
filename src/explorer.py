import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose, Quaternion
from robots_exercise_3.msg import Heatmap
import util
import random as rand
import cv2

crop_left = 2900
crop_right = 3100
crop_top = 2900
crop_bottom = 3100
crop_width = crop_right - crop_left
crop_height = crop_bottom - crop_top

class Explorer:

    def __init__(self, map_model):

        self.map_model = map_model
        self.map = self.load_map()

        self.heatmap = np.ones((self.map_model.map_width, self.map_model.map_height))

        self.decay = 0.97
        self.range = 200
	
        self.visualise = True #rospy.get_param("/visualise")
        if self.visualise:
            self.redraw()
        
        self.walkable_space = [(x, y) for x in range(0, self.map_model.map_width) for y in range(0, self.map_model.map_height) if
                               self.map[x, y] == 1]

    def get_h_map(self):
        return self.heatmap[crop_left:crop_right, crop_top:crop_bottom]

    def max_min(self, arr):
        return np.amax(arr), np.amin(arr)

    def update_map(self, pose):
        rospy.logwarn("Updating map")
        (x, y) = util.pose_to_map_coords(self.map_model, pose) 
        a = multivariate_normal.pdf(np.linspace(0, crop_width, num=crop_width), mean=pose.position.x - crop_left,
                                    cov=self.range)
        b = multivariate_normal.pdf(np.linspace(0, crop_height, num=crop_height), mean=pose.position.y - crop_top,
                                    cov=self.range)
        m = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
        self.heatmap[crop_left:crop_right, crop_top:crop_bottom] *= self.decay
        self.heatmap[crop_left:crop_right, crop_top:crop_bottom] += m
        rospy.logwarn("Calling redraw")
        #msg = Heatmap()
        #crop_map = self.get_h_map()
        #msg.width = crop_map.shape[0]
        #msg.height = crop_map.shape[1]
        #msg.data = crop_map.flatten()
        #self.map_publisher.publish(msg)
        self.redraw()
        rospy.logwarn("Redrawn")

    def least_space(self):
        (x, y) = self.walkable_space[0]
        min_val = self.heat_map[x, y]
        for (tx, ty) in self.walkable_space:
            if self.walkable_space[tx, ty] < min_val:
                min_val = self.walkable_space[tx, ty]
                (x, y) = (tx, ty)
        return (x, y)

    def load_map(self):
        return np.array([min(i, 1) for i in self.map_model.occupancy.data]).reshape((self.map_model.map_height, self.map_model.map_width))

    def redraw(self):
        if self.visualise:
            rospy.logwarn("Redrawing post-visualise check")
            high, low = self.max_min(self.get_h_map())
            mod = cv2.normalize(self.get_h_map(), None, 0, 255, cv2.NORM_MINMAX)
            print(mod)
            #mod = np.zeros((500,500), np.uint8)
            #cv2.imshow("Exploration map", mod)
            rospy.logwarn("done imshow")
            #cv2.waitKey()
            rospy.logwarn("Done")

    def next_waypoint(self):
        """
        Determine the next waypoint to navigate to
        :param: avail_space_model: The space model from which to select a waypoint
        :return: Random point on the map
        """
        while True:
            x, y = util.map_coords_to_world(rand.uniform(0, self.map_model.map_width - 1),
                                            rand.uniform(0, self.map_model.map_height - 1),
                                            self.map_model)
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y

            if not util.map_pose_occupied(new_pose, self.map_model):
                rand_a = rand.uniform(0, 2 * math.pi)
                new_pose.orientation = util.rotateQuaternion(q_orig=Quaternion(0, 0, 0, 1),
                                                             yaw=rand_a)

                rospy.loginfo("Target pose created => x: {}, y: {}".format(x, y))
                return new_pose
