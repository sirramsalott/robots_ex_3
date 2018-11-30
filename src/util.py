import math
import time
from geometry_msgs.msg import Quaternion


def timed(fn):
    """ Decorator to time functions. For debugging time critical code """

    def timed(*args, **kwargs):
        t = time.time()
        print
        "[", fn, __name__, "]Start: ", t
        ret = fn(*args, **kwargs)
        print
        "[", fn, __name__, "]End:", time.time(), " = = = ", time.time() - t
        return ret

    return timed


def multiply_quaternions(qa, qb):
    """
    Multiplies two quaternions to give the roation of qb by qa
    """
    combined = Quaternion()
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # quaternion. Order is important! Original orientation is the second
    # argument rotation which will be applied to the quaternion is the first
    # argument.
    return multiply_quaternions(q_headingChange, q_orig)


def pose_to_map_coords(sensor_model, pose):
    """
    Convert a world pose into map coords in the space of the sensor model
    :param sensor_model: The sensor model containing the map to project into
    :param pose: The pose to convert
    :return: The map x and y values
    """
    ox = pose.position.x
    oy = pose.position.y

    map_x = math.floor((
                               ox - sensor_model.map_origin_x) / sensor_model.map_resolution + 0.5) + sensor_model.map_width / 2
    map_y = math.floor((
                               oy - sensor_model.map_origin_y) / sensor_model.map_resolution + 0.5) + sensor_model.map_height / 2
    return map_x, map_y


def map_coords_occupied(coords, map_model):
    """
    Is the cell of the map_model given by the coordinates occupied
    :param coords: The coordinates to check
    :param map_model: The map model to use
    :return: True if the cell is occupied
    """
    (x, y) = coords
    threshold = 80
    prob_occupied = map_model.data[int(x + y * map_model.occupancy.info.width)]
    return prob_occupied == -1 or prob_occupied > threshold


def map_pose_occupied(pose, map_model):
    """
    Is the cell of the map_model given by the pose occupied
    :param pose: The pose to check
    :param map_model: The map_model to use
    :return: True if the cell is occupied
    """
    x, y = pose_to_map_coords(map_model, pose)
    return map_coords_occupied((x, y), map_model)


def map_coords_to_world(map_x, map_y, map_model):
    """
    Project a pair of map coordinates into the world space
    :param map_x: The x map coord
    :param map_y: The y map coord
    :param map_model: The model from which the coordinates should be projected
    :return:
    """
    x = map_model.map_resolution * (
            map_x - map_model.map_width / 2) + map_model.map_origin_x
    y = map_model.map_resolution * (
            map_y - map_model.map_height / 2) + map_model.map_origin_y

    return x, y
