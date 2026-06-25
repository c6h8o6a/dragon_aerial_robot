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

class Decide_pos():
    def __init__(self):
        x,y,z,roll,pitch,yaw=rospy.get_param("~pos")
        self.rate = rospy.Rate(40)  
        self.flight_nav_msg = FlightNav()
        self.nav_pub = rospy.Publisher(f"/gimbalrotor/uav/nav", FlightNav, queue_size=1)
        self.position = np.array([x,y,z]) #[x, y, z]
        self.attitude = np.array([roll,pitch,yaw]) #[roll, pitch, yaw]
    def main(self):
        self.flight_nav_msg.target = FlightNav.COG
        self.flight_nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.flight_nav_msg.pos_xy_nav_mode =2# FlightNav.POS_VEL_MODE
        self.flight_nav_msg.yaw_nav_mode = 2#FlightNav.POS_VEL_MODE
        self.flight_nav_msg.target_pos_x,self.flight_nav_msg.target_pos_y,self.flight_nav_msg.target_pos_z=self.position
        self.flight_nav_msg.target_roll,self.flight_nav_msg.target_pitch,self.flight_nav_msg.target_yaw=self.attitude
        while self.nav_pub.get_num_connections() == 0 and not rospy.is_shutdown():
          self.rate.sleep()
        for i in range(80):
          self.flight_nav_msg.header.stamp = rospy.Time.now()
          self.nav_pub.publish(self.flight_nav_msg)
        print(self.flight_nav_msg)
        self.rate.sleep()
        
          
if __name__ == "__main__":
    rospy.init_node("decide_pos_node")
    decide_pos = Decide_pos()
    print("Start")
    decide_pos.main()
    print("End")
