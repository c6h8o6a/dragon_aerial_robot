#!/usr/bin/env python

import os, tf, math, rospy
import xml.etree.ElementTree as ET
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform, Quaternion, Pose, Point
from std_srvs.srv import SetBool
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel

def euler_to_quaternion(roll, pitch, yaw):
    # qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    # qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    # qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    # qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

def spawn_object(module_name, model_path, link_name):        
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        tree = ET.parse(model_path)
        root = tree.getroot()
        pose_element = root.find('.//pose')
        position_element = root.find('.//position')
        
        if pose_element is not None:
            pose_values = list(map(float, pose_element.text.split()))
            position_values = list(map(float, position_element.text.split()))
            x, y, z = pose_values[0:3]
            roll, pitch, yaw = pose_values[3:6]
            x+=position_values[0]
            y+=position_values[1]
            z+=position_values[2]
            yaw+=math.pi
            position_x, position_y, position_z = position_values[0:3]
    
            pose = Point(x, y, z)
            position = Point(position_x, position_y, position_z)
            orientation = euler_to_quaternion(roll, pitch, yaw)
            initial_pose = Pose(pose, orientation)
            initial_position = Pose(position, orientation)            
            print(f"\033[1;32m[Msg] Initial pose/position set.")

        else:
            print("\033[1;91m[Warn] No <pose> / <position> tag found in the model file.\033[0m")
            return None, None, None
        inertial_element = root.find('.//inertial')
        inertia_element = root.find('.//inertia')
        if inertia_element is not None:
            inertia = {}
            inertia['m'] = float(inertial_element.find('./mass').text)
            inertia['ixx'] = float(inertia_element.find('./ixx').text)
            inertia['ixy'] = float(inertia_element.find('./ixy').text)
            inertia['ixz'] = float(inertia_element.find('./ixz').text)
            inertia['iyy'] = float(inertia_element.find('./iyy').text)
            inertia['iyz'] = float(inertia_element.find('./iyz').text)
            inertia['izz'] = float(inertia_element.find('./izz').text)

            print(f"\033[1;32m[Msg] Inertia set.\033[0m")
        else:
            print("\033[1;91m[Warn] No <inertia> tag found in the model file.\033[0m")
            return None, None, None
            
        config = {
            'module_name': module_name,
            'transform': Transform(translation=pose, rotation=orientation),
            'inertia': inertia
        }

        with open(model_path, 'r') as model_file:
            model_sdf = model_file.read()

        return model_sdf, initial_position, config

    except rospy.ServiceException as e:
        rospy.logerr("\033[1;91m[Warn] Spawn service call failed: %s\033[0m" % e)
        return None, None, None

def delete_object(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        response = delete_model_service(model_name)
        if response.success:
            print(f"\033[1;32m[Msg] {model_name} has been deleted successfully!\033[0m")
        else:
            print(f"\033[1;91m[Warning] Failed to delete {model_name}.\033[0m")
    except rospy.ServiceException as e:
        print(f"\033[1;91m[Warning] Service call failed: {e}\033[0m")
    
def call_add_extra_module(action, link_name, config):
    rospy.wait_for_service('/gimbalrotor/add_extra_module')
    try:
        add_extra_module = rospy.ServiceProxy('/gimbalrotor/add_extra_module', AddExtraModule)

        request = AddExtraModuleRequest()
        request.action = action
        request.module_name = config['module_name']
        request.parent_link_name = link_name
        request.transform = config['transform']
        request.inertia.m = config['inertia']['m']
        request.inertia.ixx = config['inertia']['ixx']
        request.inertia.ixy = config['inertia']['ixy']
        request.inertia.ixz = config['inertia']['ixz']
        request.inertia.iyy = config['inertia']['iyy']
        request.inertia.iyz = config['inertia']['iyz']
        request.inertia.izz = config['inertia']['izz']

        response = add_extra_module(request)
        if response.status:
            print("""
########################################################################################
\033[1mModule added/deleted successfully!\033[0m
########################################################################################
            """)
            return True
        else:
            print("""
########################################################################################
\033[1mFailed to add/delete module!\033[0m
########################################################################################
            """)
            return False
    except rospy.ServiceException as e:
        print("""
########################################################################################
\033[1mService call failed!: \033[0m
########################################################################################
        """)
        return False

pub = rospy.Publisher('/gimbalrotor/docking_cmd', Bool, queue_size=10)
    
def send_docking_command(is_docking):
    rospy.sleep(0.5)
    pub.publish(is_docking)
    rospy.sleep(0.5)

def main():
    rospy.init_node('add_module_client')
    rospy.sleep(1)

    while True:
        print("\033[1mSelect parent link, plus or minus: \033[0m")
        link = input().strip().lower()
        if link == "plus":
            link_name = "gimbal_link1"
            break
        elif link == "minus":
            link_name = "base_link"
            break
        else:
            print(f"\033[1;91m[Warn] Wrong selection. Please select a valid link.\033[0m")
            continue

    # link_name = "dummy_gripper"

    while True:
        print("\033[1mEnter the module name: \033[0m")

        module_name = input().strip().lower()
        model_path = f'/home/newuser/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/gimbalrotor/src/model/{module_name}.sdf'
        if not os.path.exists(model_path):
            print(f"\033[1;91m[Warning] '{module_name}' model not found. Please enter a valid module name.\033[0m")
            continue
        else:
            model_sdf, initial_position, config = spawn_object(module_name, model_path, "hook_tool_link")
            if model_sdf is None:
                continue
            break

    while True:
        print("\033[1mEnter add or delete: \033[0m")
        operation = input().strip().lower()        
        if operation == "add":                
            action = 1
            send_docking_command(True)
            try:
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                resp=spawn_model(module_name, model_sdf, '', initial_position, "hook_tool_link")
                print(resp)    
                if call_add_extra_module(action, link_name, config):
                    print(f"\033[1;32m[Msg] {module_name} model spawned successfully!\033[0m")
                    print("\033[1;32m[Msg] Attachment on!\033[0m")
                    break
                else:
                    print("\033[1;91m[Error] Failed to add module. Please try again.\033[0m")
                    continue
            except rospy.ServiceException as e:
                print(f"\033[1;91m[Warn] Spawn service call failed: {e}\033[0m")
                continue

        elif operation == "delete":
            action = -1
            send_docking_command(False)
            print("\033[1;32m[Msg] Attachment off!\033[0m")
            if call_add_extra_module(action, link_name, config):
                rospy.sleep(2)
                try:
                    delete_object(module_name)
                    break
                except rospy.ServiceException as e:
                    rospy.logerr("\033[1;91m[Warn] Despawn service call failed: %s\033[0m" % e)
                    continue
                
        else:
            print("\033[1;91mInvalid operation. Please enter 'add' or 'delete'.\033[0m")

        
if __name__ == "__main__":
    main()
