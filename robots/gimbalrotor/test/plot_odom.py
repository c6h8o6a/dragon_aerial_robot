#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from aerial_robot_msgs.msg import FlightNav
import math
import numpy as np
class OdomPlotter:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/gimbalrotor/uav/cog/odom")
        self.duration = rospy.get_param("~duration", 10.0)
        if rospy.has_param("~pos"):
            x,y,z,roll,pitch,yaw=rospy.get_param("~pos")
            self.position = np.array([x,y,z]) #[x, y, z]
            self.attitude = np.array([roll,pitch,yaw]) #[roll, pitch, yaw]
            r,p,yaw=math.degrees(roll),math.degrees(pitch),math.degrees(yaw)
            print(f"x:{x}")
            print(f"y:{y}")
            print(f"z:{z}")
            print(f"r:{r}")
            print(f"p:{p}")
            print(f"y:{yaw}")
        
        self.rate=rospy.Rate(40)
        self.flight_nav_msg = FlightNav()
        self.t0 = None

        self.ts = []
        self.xs = []
        self.ys = []
        self.zs = []
        self.rolls = []
        self.pitchs = []
        self.yaws = []
        self.exs = []
        self.eys = []
        self.ezs = []
        self.erolls = []
        self.epitchs = []
        self.eyaws = []
        self.target_pos=[x,y,z]
        self.target_ori=[roll,pitch,yaw]
        self.sub = rospy.Subscriber(self.topic, Odometry, self.cb)
        self.nav_pub = rospy.Publisher(f"/gimbalrotor/uav/nav", FlightNav, queue_size=1)
        
    def cb(self, msg):
        now = rospy.Time.now()

        if self.t0 is None:
            self.t0 = now

        t = (now - self.t0).to_sec()

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        self.ts.append(t)
        self.xs.append(p.x)
        self.ys.append(p.y)
        self.zs.append(p.z)
        self.rolls.append(math.degrees(roll))
        self.pitchs.append(math.degrees(pitch))
        self.yaws.append(math.degrees(yaw))

        self.exs.append(-p.x+self.target_pos[0])
        self.eys.append(-p.y+self.target_pos[1])
        self.ezs.append(-p.z+self.target_pos[2])
        self.erolls.append(math.degrees(self.target_ori[0])-math.degrees(roll))
        self.epitchs.append(math.degrees(self.target_ori[1])-math.degrees(pitch))
        self.eyaws.append(math.degrees(self.target_ori[2])-math.degrees(yaw))

    def run(self):
        if rospy.has_param("~pos"):
            self.move()
        print("recording topic:", self.topic)
        print("duration:", self.duration, "sec")

        while not rospy.is_shutdown():
            if self.t0 is not None:
                elapsed = (rospy.Time.now() - self.t0).to_sec()
                if elapsed >= self.duration:
                    break
            self.rate.sleep()

        print("record finished")
        print("samples:", len(self.ts))

        if len(self.ts) == 0:
            print("no data received")
            return
        self.plot()

    def plot(self):
        # x, y, z vs time
        plt.figure()
        plt.plot(self.ts, self.xs, label="x")
        plt.plot(self.ts, self.ys, label="y")
        plt.plot(self.ts, self.zs, label="z")
        plt.xlabel("time [s]")
        plt.ylabel("position [m]")
        plt.title("Position")
        plt.grid(True)
        plt.legend()

        # roll, pitch, yaw vs time
        plt.figure()
        plt.plot(self.ts, self.rolls, label="roll")
        plt.plot(self.ts, self.pitchs, label="pitch")
        plt.plot(self.ts, self.yaws, label="yaw")
        plt.xlabel("time [s]")
        plt.ylabel("angle [deg]")
        plt.title("Attitude")
        plt.grid(True)
        plt.legend()

        #e x, y, z vs time
        plt.figure()
        plt.plot(self.ts, self.exs, label="ex")
        plt.plot(self.ts, self.eys, label="ey")
        plt.plot(self.ts, self.ezs, label="ez")
        plt.xlabel("time [s]")
        plt.ylabel("position [m]")
        plt.title("Position")
        plt.grid(True)
        plt.legend()

        #e roll, pitch, yaw vs time
        plt.figure()
        plt.plot(self.ts, self.erolls, label="eroll")
        plt.plot(self.ts, self.epitchs, label="epitch")
        plt.plot(self.ts, self.eyaws, label="eyaw")
        plt.xlabel("time [s]")
        plt.ylabel("angle [deg]")
        plt.title("Attitude")
        plt.grid(True)
        plt.legend()
        
        # xy trajectory
        plt.figure()
        plt.plot(self.xs, self.ys)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title("XY trajectory")
        plt.axis("equal")
        plt.grid(True)

        plt.show()

    def move(self):
        self.flight_nav_msg.target = FlightNav.COG
        self.flight_nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.flight_nav_msg.pos_xy_nav_mode =4# FlightNav.POS_VEL_MODE
        self.flight_nav_msg.yaw_nav_mode = 4#FlightNav.POS_VEL_MODE
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
    rospy.init_node("plot_odom")
    plotter = OdomPlotter()
    plotter.run()
