#! /usr/bin/env python

import math

TRACK_FACE_LEFT = 0.1
TRACK_FACE_RIGHT = 0.1
TRACK_FACE_FORWARD = 0.4


def pose_to_map_coords(self, pose):
    ox = pose.position.x
    oy = pose.position.y

    map_x = math.floor((
                               ox - self.sensor_model.map_origin_x) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_width / 2
    map_y = math.floor((
                               oy - self.sensor_model.map_origin_y) / self.sensor_model.map_resolution + 0.5) + self.sensor_model.map_height / 2

    return int(math.floor(map_x)), int(math.floor(map_y))


def map_coords_to_world(self, map_x, map_y):
    x = map.map_resolution * (
            map_x - map.map_width / 2) + self.sensor_model.map_origin_x
    y = map.map_resolution * (
            map_y - map.map_height / 2) + self.sensor_model.map_origin_y

    return x, y


def map_cell_occupied(self, pose):
    x, y = pose_to_map_coords(pose)
    threshold = 80
    prob_occupied = self.occupancy_map.data[x + y * self.occupancy_map.info.width]

    return prob_occupied == -1 or prob_occupied > threshold
