import math
import time
from geometry_msgs.msg import Quaternion

def timed(fn):
    """ Decorator to time functions. For debugging time critical code """
    def timed(*args,  **kwargs):
        t = time.time()
        print "[", fn, __name__, "]Start: ",  t
        ret =  fn(*args, **kwargs)
        print "[", fn, __name__, "]End:", time.time(),  " = = = ",  time.time() - t
        return ret
    return timed

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