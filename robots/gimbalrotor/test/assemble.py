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


class TrrAssembly():
    def __init__(self):        
        self.flight_nav_msg = FlightNav()
        self.rate = rospy.Rate(40)  # 40Hz
        self.dis=0.5
        self.yawdiff=math.pi/18
        self.nav_pub = rospy.Publisher("/gimbalrotor/uav/nav", FlightNav, queue_size=1)
        self.gimbalrotor_pose_sub = rospy.Subscriber("/gimbalrotor/uav/cog/odom",Odometry, self.poseCb_self)
        self.hook_pose_sub=rospy.Subscriber("/gazebo/model_states",ModelStates,self.hookCb)
        self.gimbalrotor_position = np.zeros(3) #[x, y, z]
        self.gimbalrotor_attitude = np.zeros(3) #[roll, pitch, yaw]
        self.hook_position=np.zeros(3)
        self.hook_attitude=np.zeros(3)
        self.phase=1
        self.phase2_sent=False
        self.vel=np.zeros(3)
        self.phase3_sent=False
        self.target=np.zeros(3)
        self.targetyaw=0
        self.duration=3
        
        self.phase3_start = None
        self.ctrl_mode_pub = rospy.Publisher("/gimbalrotor/teleop_command/ctrl_mode",Int8,queue_size=1)

        self.got_self = False
        self.got_hook = False

        '''
        self.flight_nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        flight_nav_msg.target_vel_x = xxx
        '''

    def main(self):
        self.reset_before_start()
        while not rospy.is_shutdown():
          rot = np.array([[np.cos(math.pi/2), -np.sin(math.pi/2),0],
                             [np.sin(math.pi/2), np.cos(math.pi/2),0],
                             [0,0,0]])
          cvec=self.hook_position-self.gimbalrotor_position
          cvec[2]=0
          cvec/=np.linalg.norm(cvec)
          rvec=np.dot(rot,cvec)
          
          if self.phase==1:
            self.flight_nav_msg.target = FlightNav.COG
            self.flight_nav_msg.control_frame = FlightNav.WORLD_FRAME
            self.flight_nav_msg.pos_xy_nav_mode =1# FlightNav.POS_MODE
            self.flight_nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            self.flight_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            
            self.flight_nav_msg.target_vel_x = rvec[0]*1#+cvec[0]*0.5#-math.cos(self.ninja2_attitude[2])*1
            self.flight_nav_msg.target_vel_y = rvec[1]*1#+cvec[1]*0.5#-math.sin(self.ninja2_attitude[2])*1
            # if (self.gimbalrotor_position[0]-self.hook_position[0])**2 + (self.gimbalrotor_position[1]-self.hook_position[1])**2>(1)**2:
            #    self.flight_nav_msg.target_pos_x = self.gimbalrotor_position[0]+rvec[0]*1+cvec[0]*0.8
            #    self.flight_nav_msg.target_pos_y = self.gimbalrotor_position[1]+rvec[1]*1+cvec[1]*0.8 
            #self.flight_nav_msg.target_yaw = self.hook_attitude[2]+math.pi
            print("1",self.flight_nav_msg.target_pos_x,self.flight_nav_msg.target_pos_y)
            self.flight_nav_msg.header.stamp = rospy.Time.now()
            self.nav_pub.publish(self.flight_nav_msg)
            if self.angle_diff(np.arctan2(-cvec[1],-cvec[0]),self.hook_attitude[2])<math.pi/6:
                self.phase=2
                self.flight_nav_msg.target = FlightNav.COG
                self.flight_nav_msg.control_frame = FlightNav.WORLD_FRAME
                self.flight_nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                self.flight_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
          elif self.phase==2:
            print( "phase2_sent =", self.phase2_sent,np.arctan2(-cvec[1],-cvec[0]),self.hook_attitude[2])
            if not self.phase2_sent:
              self.flight_nav_msg.target = FlightNav.COG
              self.flight_nav_msg.control_frame = FlightNav.WORLD_FRAME
              self.flight_nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
              self.flight_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
              self.flight_nav_msg.target_pos_x = self.hook_position[0]+math.cos(self.hook_attitude[2])*self.dis
              self.flight_nav_msg.target_pos_y = self.hook_position[1]+math.sin(self.hook_attitude[2])*self.dis
              self.flight_nav_msg.target_yaw = self.hook_attitude[2]+math.pi
              self.target=np.array([self.hook_position[0]+math.cos(self.hook_attitude[2])*self.dis,self.hook_position[1]+math.sin(self.hook_attitude[2])*self.dis,0])
              #self.targetyaw=math.atan2(self.target[1],self.target[0])
              if self.nav_pub.get_num_connections() > 0:
                self.phase2_sent=True
                for i in range(40):
                  self.flight_nav_msg.header.stamp = rospy.Time.now()
                  self.nav_pub.publish(self.flight_nav_msg)
            print("2",self.target,self.flight_nav_msg.pos_xy_nav_mode)
            for i in range(40):
                  self.flight_nav_msg.header.stamp = rospy.Time.now()
                  self.nav_pub.publish(self.flight_nav_msg)
            if self.lateral_error_to_line(self.gimbalrotor_position,self.target, self.hook_position)<0.05 and  self.angle_diff(self.gimbalrotor_attitude[2],self.flight_nav_msg.target_yaw)<self.yawdiff and (self.gimbalrotor_position[0]-self.target[0])**2 + (self.gimbalrotor_position[1]-self.target[1])**2<0.1:
              # (self.gimbalrotor_position[0]-self.flight_nav_msg.target_pos_x)**2 + (self.gimbalrotor_position[1]-self.flight_nav_msg.target_pos_y)**2<(0.1)**2  
              #self.angle_diff(self.ninja1_attitude[2],self.targetyaw)<self.yawdiff
              # self.flight_nav_msg.target_yaw = self.ninja2_attitude[2]
              # while self.nav_pub.get_num_connections() == 0 
              # self.nav_pub.publish(self.flight_nav_msg)
              self.phase=3
              self.phase3_sent=True
              self.vel=[1,0,0]
              self.phase3_start=rospy.Time.now()
          elif self.phase==3: #and (self.ninja1_position[0]-self.)**2 + (self.ninja1_position[1]-self.ninja2_position[1])**2<self.dis**2 and   abs(self.ninja1_attitude[2]-self.ninja2_attitude[2])<self.yawdiff :
           if self.phase3_sent:
             self.flight_nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE                   
             self.flight_nav_msg.control_frame = FlightNav.LOCAL_FRAME
             self.flight_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
             self.flight_nav_msg.target_yaw = self.hook_attitude[2]+math.pi
             # if (self.gimbalrotor_position[0]-self.hook_position[0])**2 + (self.gimbalrotor_position[1]-self.hook_position[1])**2>=(0.85)**2:
             if True:
              print("3-1")
              self.flight_nav_msg.target_vel_x= self.vel[0]*0.3
              self.flight_nav_msg.target_vel_y= self.vel[1]*0.3
              elapsed = (rospy.Time.now() - self.phase3_start).to_sec()
              if elapsed > self.duration:
                  self.flight_nav_msg.target_vel_x= 0
                  self.flight_nav_msg.target_vel_y= 0
                  self.flight_nav_msg.header.stamp = rospy.Time.now()
                  self.nav_pub.publish(self.flight_nav_msg)
                  self.rate.sleep()
                  self.phase=4
                  continue
              self.flight_nav_msg.header.stamp = rospy.Time.now()
              self.nav_pub.publish(self.flight_nav_msg)
              self.rate.sleep()

             else:
               print("3-3")
               self.phase=4
           else:
            self.flight_nav_msg.target_pos_x=self.hook_position[0]-2.575*math.cos(self.hook_attitude[2])
            self.flight_nav_msg.target_pos_y=self.hook_position[1]+2.575*math.sin(self.hook_attitude[2])
            self.flight_nav_msg.target_yaw=self.hook_attitude[2]
            if self.nav_pub.get_num_connections() > 0:
                 for i in range(20):
                   self.flight_nav_msg.header.stamp = rospy.Time.now()
                   self.nav_pub.publish(self.flight_nav_msg)
            #self.flight_nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
            #self.flight_nav_msg.control_frame = FlightNav.LOCAL_FRAME
            self.vel=np.array([1,0,0])#cvec
            #self.phase3_sent=True
            print("3-0")
          else:
           print("4,debug")
           self.reset_before_start()
           exit()
          self.rate.sleep()
        
    def hookCb(self, msg):
        if "hook" not in msg.name:
            return
        
        idx = msg.name.index("hook")
        pose = msg.pose[idx]
        
        self.hook_position[0] = pose.position.x
        self.hook_position[1] = pose.position.y
        self.hook_position[2] = pose.position.z
        
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        
        roll, pitch, yaw = euler_from_quaternion(q)
        
        self.hook_attitude[0] = roll
        self.hook_attitude[1] = pitch
        self.hook_attitude[2] = yaw 
        self.got_hook = True
        
    def poseCb_self(self,msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        (roll,pitch,yaw) = euler_from_quaternion(quaternion)
        self.gimbalrotor_position[0] = msg.pose.pose.position.x
        self.gimbalrotor_position[1] = msg.pose.pose.position.y
        self.gimbalrotor_attitude[2] = yaw
        self.got_self = True
    def angle_diff(self,a, b):
      d = a - b
      return abs(math.atan2(math.sin(d), math.cos(d)))
    def lateral_error_to_line(self, robot_pos, target_pos, hook_pos):
        # xy平面だけ使う
        target_xy = np.array([target_pos[0], target_pos[1]])
        hook_xy   = np.array([hook_pos[0], hook_pos[1]])
        robot_xy  = np.array([robot_pos[0], robot_pos[1]])

        # target -> hook の方向ベクトル
        d = hook_xy - target_xy
        norm = np.linalg.norm(d)
        
        if norm < 1e-6:
            print("min:1e-6")
            return 999.0

        d = d / norm
        # d に垂直な単位ベクトル
        n = np.array([-d[1], d[0]])

        # target から見た機体位置誤差
        e = robot_xy - target_xy

        # 垂直方向の誤差
        lateral_error = abs(np.dot(e, n))
        return lateral_error

    
    def reset_before_start(self):
        while (not self.got_self or not self.got_hook) and not rospy.is_shutdown():
            print("waiting odom/hook...")
            self.rate.sleep()

        # 内部制御をPOSに戻す
        # ctrl = Int8()
        # ctrl.data = 0

        for i in range(20):
            #self.ctrl_mode_pub.publish(ctrl)
            self.rate.sleep()

            # 現在位置を明示的にPOS保持
            msg = FlightNav()
            msg.target = FlightNav.COG
            msg.control_frame = FlightNav.WORLD_FRAME
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            msg.yaw_nav_mode = FlightNav.POS_MODE
            
            msg.target_pos_x = self.gimbalrotor_position[0]
            msg.target_pos_y = self.gimbalrotor_position[1]
            msg.target_yaw = self.gimbalrotor_attitude[2]


            for i in range(2):
                msg.header.stamp = rospy.Time.now()
                self.nav_pub.publish(msg)
                self.rate.sleep()
                self.flight_nav_msg = msg
                print("reset before start done")
if __name__ == "__main__":
    rospy.init_node("trr_assembly_node")
    trr_assembly = TrrAssembly()
    trr_assembly.main()
