"""
map_model.py
Provides a SensorModel class to calculate particle weights.
"""
import rospy
import math

PI_OVER_TWO = math.pi / 2


class MapModel(object):

    def __init__(self, provided_map):
        """
        Set the map that this model should use when calculating expected
        laser readings.
        
        :Args:
            | provided_map (sensor_msgs.msg.OccupancyGrid): the map to use
        """
        # Map data
        self.occupancy = provided_map
        self.map_width = provided_map.info.width
        self.map_height = provided_map.info.height
        self.map_resolution = provided_map.info.resolution  # in m per pixel
        self.map_data = provided_map.data
        self.map_origin_x = (provided_map.info.origin.position.x +
                             (self.map_width / 2.0) * self.map_resolution)
        self.map_origin_y = (provided_map.info.origin.position.y +
                             (self.map_height / 2.0) * self.map_resolution)
        rospy.loginfo("Sensor model occupancy map set.")
