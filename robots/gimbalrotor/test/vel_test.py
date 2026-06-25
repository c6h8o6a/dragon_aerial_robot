#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from aerial_robot_msgs.msg import FlightNav


class VelNavTest:
    def __init__(self):
        self.rate = rospy.Rate(40)

        self.nav_pub = rospy.Publisher(
            "/gimbalrotor/uav/nav",
            FlightNav,
            queue_size=1
        )

        # パラメータ
        self.duration = rospy.get_param("~duration", 2.0)

        # LOCAL_FRAMEでのxy速度 [m/s]
        self.vel_x = rospy.get_param("~vel_x", 1)
        self.vel_y = rospy.get_param("~vel_y", 0.0)

        # yawだけ維持したい場合
        self.target_yaw = rospy.get_param("~target_yaw", 0.0)

        self.msg = FlightNav()

    def run(self):
        print("waiting subscriber...")
        while self.nav_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            self.rate.sleep()

        print("start velocity command")
        print("vel_x:", self.vel_x)
        print("vel_y:", self.vel_y)
        print("duration:", self.duration)

        self.msg.target = FlightNav.COG

        # 機体座標系で速度指令
        self.msg.control_frame = FlightNav.LOCAL_FRAME

        # xyだけ速度制御
        self.msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        self.msg.target_vel_x = self.vel_x
        self.msg.target_vel_y = self.vel_y

        # zは操作しない

        # yawは維持
        self.msg.yaw_nav_mode = FlightNav.POS_MODE
        self.msg.target_yaw = self.target_yaw

        start = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > self.duration:
                break

            self.msg.header.stamp = rospy.Time.now()
            self.nav_pub.publish(self.msg)
            self.rate.sleep()

        print("stop velocity command")

        # xy速度を0にする
        self.msg.target_vel_x = 0.0
        self.msg.target_vel_y = 0.0
        self.msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        self.msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION

        for i in range(40):
            self.msg.header.stamp = rospy.Time.now()
            self.nav_pub.publish(self.msg)
            self.rate.sleep()

        print("finished")


if __name__ == "__main__":
    rospy.init_node("vel_nav_test")
    node = VelNavTest()
    node.run()
