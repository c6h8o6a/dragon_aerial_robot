#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int8
from spinal.msg import ServoControlCmd


class Push():
    def __init__(self):        
        self.rate = rospy.Rate(40)  # 40Hz
        self.angle = rospy.get_param("~angle", 2048)
        self.servo_target_pub = rospy.Publisher("/gimbalrotor/servo/target_states",ServoControlCmd,queue_size=1)

    def main(self):
        self.publish_servo_angles([4],[self.angle],20)

    def publish_servo_angles(self, indices, angles, repeat=20):
        while self.servo_target_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("waiting for servo target subscriber...")
            self.rate.sleep()
        msg = ServoControlCmd()
        msg.index = indices
        msg.angles = angles
        for i in range(repeat):
            self.servo_target_pub.publish(msg)
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("one_push__node")
    push = Push()
    push.main()
