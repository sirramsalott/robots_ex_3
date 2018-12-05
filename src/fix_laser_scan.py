#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math

class Fixer:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/broken_scan", LaserScan, self.reading, queue_size=100)
        self.pub = rospy.Publisher("/base_scan", LaserScan, queue_size=100)
        rospy.spin() 
    
    def fix(self, x, m):
        if math.isnan(x):
            return m - 0.1
        else:
            return x

    def reading(self, msg):
        resp = LaserScan()
        resp.header = msg.header
        resp.angle_min = msg.angle_min
        resp.angle_max = msg.angle_max
        resp.angle_increment = msg.angle_increment
        resp.time_increment = msg.time_increment
        resp.scan_time = msg.scan_time
        resp.range_min = msg.range_min
        resp.range_max = msg.range_max
        resp.ranges = [self.fix(x, msg.range_max) for x in msg.ranges]
        resp.intensities = msg.intensities
        self.pub.publish(resp)


if __name__ == '__main__':
    rospy.init_node('fix_laser_scan')
    fixer = Fixer()
