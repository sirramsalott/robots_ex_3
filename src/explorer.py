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

crop_left = 1400
crop_right = 2400
crop_top = 1400
crop_bottom = 2600
crop_width = crop_right - crop_left
crop_height = crop_bottom - crop_top


class Explorer:

    def __init__(self, map_model):

        self.map_model = map_model
        self.map = self.load_map()
        self.display_map_G = self.map[crop_left:crop_right, crop_bottom:crop_top] * 255
        self.heatmap = np.zeros((self.map_model.map_width, self.map_model.map_height))

        self.decay = 0.9993
        self.range = 700
        self.lower_threshold = 0.01
        self.update_count = 0
        
        a = self.map[crop_left:crop_right, crop_top:crop_bottom]
        high, low = self.max_min(a)
        mod = cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imwrite("/home/robot/avail_map.png", mod)

        self.visualise = rospy.get_param("/visualise")
        if self.visualise:
            self.redraw()
        
        self.walkable_space = [(x, y) for x in range(0, self.map_model.map_width) for y in range(0, self.map_model.map_height) if
                               self.map[y, x] != 1]

    def get_h_map(self):
        return self.heatmap[crop_left:crop_right, crop_top:crop_bottom]

    def max_min(self, arr):
        return np.amax(arr), np.amin(arr)

    def update_map(self, pose):
        (x, y) = util.pose_to_map_coords(self.map_model, pose) 
        a = multivariate_normal.pdf(np.linspace(0, crop_width, num=crop_width), mean=x - crop_left,
                                    cov=self.range)
        maxi = np.amax(a)
        b = multivariate_normal.pdf(np.linspace(0, crop_height, num=crop_height), mean=y - crop_top,
                                    cov=self.range)
        a = np.divide(a, maxi)
        b = np.divide(b, maxi)
        m = (np.array(a)[np.newaxis]).T.dot((np.array(b)[np.newaxis]))
        self.heatmap[crop_left:crop_right, crop_top:crop_bottom] *= self.decay
        self.heatmap[crop_left:crop_right, crop_top:crop_bottom] = np.maximum(m, self.get_h_map())
        self.redraw()
        rospy.logwarn(self.update_count)
        self.update_count += 1 

    def clean_map(self):
        for (x, y) in self.walkable_space:
            if self.heatmap[x][y] < self.lower_threshold:
                self.heatmap[x][y] = 0

    def least_space(self):
        (x, y) = self.walkable_space[0]
        min_val = self.heatmap[x][y]
        p = []
        for (tx, ty) in self.walkable_space:
            if self.heatmap[tx][ty] == min_val:
                p.append((tx,ty))
            if self.heatmap[tx][ty] < min_val:
                min_val = self.heatmap[tx][ty]
                p = [(tx, ty)]
        return p[rand.randint(0,len(p) - 1)]

    def load_map(self):
        return np.array([min(i, 1) for i in self.map_model.occupancy.data]).reshape((self.map_model.map_height, self.map_model.map_width))

    def redraw(self):
        if self.visualise:
            high, low = self.max_min(self.get_h_map())
            mod = cv2.normalize(self.get_h_map(), None, 0, 255, cv2.NORM_MINMAX)
            cv2.imwrite("/home/robot/im.png", mod)


    def next_waypoint(self):
        """
        Determine the next waypoint to navigate to
        :param: avail_space_model: The space model from which to select a waypoint
        :return: Random point on the map
        """
        new_pose = Pose()
        self.clean_map()
        (x, y) = self.least_space()
        wx, wy = util.map_coords_to_world(x, y, self.map_model)
        new_pose.position.x = wx
        new_pose.position.y = wy
        rand_a = rand.uniform(0, 2 * math.pi)
        new_pose.orientation = util.rotateQuaternion(q_orig=Quaternion(0, 0, 0, 1), yaw=rand_a)
        rospy.logwarn("Target pose created => x: {}, y: {}".format(x, y))
        return new_pose

