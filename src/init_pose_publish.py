#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish():
    # Publish the robots movement commands
    print("Starting publisher")
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    msg = PoseWithCovarianceStamped()
    
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    msg.pose.pose.position.x = -2.5501832962
    msg.pose.pose.position.y = -6.95900058746
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.456655171149
    msg.pose.pose.orientation.w = 0.889643779646

    msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

    print("Publishing")
    rospy.sleep(1)
    pub.publish(msg)
    print("Published")
    


if __name__ == '__main__':
    rospy.init_node(name="init_pose_publisher", log_level=rospy.INFO)
    publish()
