#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def cb(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    quat = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quat)

    print("position")
    print("  x:", p.x)
    print("  y:", p.y)
    print("  z:", p.z)

    print("attitude [rad]")
    print("  roll :", roll)
    print("  pitch:", pitch)
    print("  yaw  :", yaw)

    print("attitude [deg]")
    print("  roll :", math.degrees(roll))
    print("  pitch:", math.degrees(pitch))
    print("  yaw  :", math.degrees(yaw))
    print("-----")

rospy.init_node("read_pose")
rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, cb)
rospy.spin()
