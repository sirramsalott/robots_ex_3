import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose
import movement_model as mm

class Explorer:

    def __init__(self, map_file):

        self.map = self.load_map(map_file)
        (self.map_width, self.map_height) = self.map.shape

        self.heatmap = np.ones((self.map_width, self.map_height))

        self.decay = 0.97
        self.range = 200

        plt.ion()
        self.redraw()
        self.walkable_space = [(x, y) for x in range(0, self.map_width) for y in range(0, self.map_height) if self.map[x,y] == 1]

    def update_map(self, pose):
        (x, y) = mm.pose_to_map_coords(pose) #TODO Fix this
        a = multivariate_normal.pdf(np.linspace(0, self.map_width, num=self.map_width), mean=pose.position.x, cov=self.range)
        b = multivariate_normal.pdf(np.linspace(0, self.map_height, num=self.map_height), mean=pose.position.y, cov=self.range)
        m = (np.array(a)[np.newaxis]).T.dot((np.arra(a)[np.newaxis]))
        self.heatmap = (self.heatmap * self.decay) + m
        self.redraw()

    def redraw(self):
        plt.imshow(self.heatmap, cmap='hot', interpolation='nearest')
        plt.draw()

    def least_space(self):
        (x, y) = self.walkable_space[0]
        min_val = self.heat_map[x,y]
        for (tx, ty) in self.walkable_space:
            if self.walkable_space[tx,ty] < min_val:
                min_val = self.walkable_space[tx, ty]
                (x, y) = (tx, ty)
        return (x,y)
    
    def load_map(self, map_file):
        
        def read_pgm(filename, byteorder='>'):

            with open(filename, 'rb') as f:
                buffer = f.read()
            try:
                header, width, height, maxval = re.search(
                    b"(^P5\s(?:\s*#.*[\r\n])*"
                    b"(\d+)\s(?:\s*#.*[\r\n])*"
                    b"(\d+)\s(?:\s*#.*[\r\n])*"
                    b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
            except AttributeError:
                raise ValueError("Not a raw PGM file: '%s'" % filename)
            return np.frombuffer(buffer,
                                    dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                                    count=int(width)*int(height),
                                    offset=len(header)
                                    ).reshape((int(height), int(width)))

        def normalise(val):
            if val > 0:
                return 1

        return normalise(read_pgm(map_file))
   
