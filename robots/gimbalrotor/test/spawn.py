#!/usr/bin/env python

import rospy

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
import tf.transformations as tft
import math

rospy.init_node("spawn_tool")
name=input("Please send name")
# SDFファイル読み込み
with open(f"/home/newuser/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/src/model/{name}.sdf", "r") as f:
    model_xml = f.read()
while True:
 n=input("Enter word")
 if n=="a":
    rospy.wait_for_service("/gazebo/spawn_sdf_model")

    spawn_model = rospy.ServiceProxy(
    "/gazebo/spawn_sdf_model",
    SpawnModel)

    pose = Pose()
    if name=="hook":
        x,y,z=0,1,0.0545+0.091
        roll, pitch, yaw = 0.0, 0.0, -math.pi / 2
    elif name=="hook2":
        x,y,z=0,1,0.091+0.05529
        roll, pitch, yaw = 0.0, 0.0, -math.pi / 2
    elif name=="box":
        x,y,z=0,1.2,0.0455
        roll, pitch, yaw = 0.0, 0.0, 0.0
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    resp = spawn_model(
    f"{name}",      # model_name
    model_xml,   # model_xml
    "",          # robot_namespace
    pose,        # initial_pose
    "world")     # reference_frame

    print(resp.success)
    print(resp.status_message)

    break
 elif n=="d":
    # 削除サービス
    rospy.wait_for_service("/gazebo/delete_model")
    delete_model = rospy.ServiceProxy(
    "/gazebo/delete_model",
    DeleteModel)

    try:
        delete_model(f"{name}")
        rospy.sleep(0.5)
    except:
        print("error_delete")
        pass
    break
else:
    print("error_word")
