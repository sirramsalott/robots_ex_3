import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose, Quaternion
import movement_model as mm
import util
import random as rand


class Explorer:

    def __init__(self, map_model):

        self.map_model = map_model
        self.map = self.load_map()

        self.heatmap = np.ones((self.map_model.map_width, self.map_model.map_height))

        self.decay = 0.97
        self.range = 200
	
        self.visualise = False #rospy.get_param("/visualise")
        if self.visualise:
            plt.ion()
            self.redraw()
        
        self.walkable_space = [(x, y) for x in range(0, self.map_model.map_width) for y in range(0, self.map_model.map_height) if
                               self.map[x, y] == 1]

    def update_map(self, pose):
        rospy.loginfo("Updating map...")
        (x, y) = util.pose_to_map_coords(self.map_model, pose)
        a = multivariate_normal.pdf(np.linspace(0, self.map_model.map_width, num=self.map_model.map_width), mean=pose.position.x,
                                    cov=self.range)
        b = multivariate_normal.pdf(np.linspace(0, self.map_model.map_height, num=self.map_model.map_height), mean=pose.position.y,
                                    cov=self.range)
        m = (np.array(a)[np.newaxis]).T.dot((np.array(a)[np.newaxis]))
        self.heatmap = (self.heatmap * self.decay) + m
        self.redraw()
        rospy.loginfo("Map Updated!")

    def redraw(self):
        if self.visualise:
            plt.imshow(self.heatmap, cmap='hot', interpolation='nearest')
            plt.show()

    def least_space(self):
        (x, y) = self.walkable_space[0]
        min_val = self.heat_map[x, y]
        for (tx, ty) in self.walkable_space:
            if self.walkable_space[tx, ty] < min_val:
                min_val = self.walkable_space[tx, ty]
                (x, y) = (tx, ty)
        return (x, y)


    def load_map(self):
        def convert_map(map_model):
            return np.array([min(i, 1) for i in map_model.occupancy.data]).reshape((map_model.map_height, map_model.map_width))

        return convert_map(self.map_model)

    def next_waypoint(self):
        """
        Determine the next waypoint to navigate to
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

                rospy.loginfo("Target porse created => x: {}, y: {}".format(x, y))
                return new_pose
