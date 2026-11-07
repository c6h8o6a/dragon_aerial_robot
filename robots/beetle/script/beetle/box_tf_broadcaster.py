#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetLinkProperties
from beetle.msg import ModuleModel

class BoxTFBroadcaster:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.last_stamp = None
        self.cog_offset = rospy.get_param(
            '~cog_offset',
            [0.0, 0.0, 0.0]
        )
        self.model_published = False
        self.module_model_pub = rospy.Publisher(
            '/beetle2/unified_control/module_model',
            ModuleModel,
            queue_size=1,
            latch=True
        )
        self.get_link_properties = rospy.ServiceProxy(
            '/gazebo/get_link_properties',
            GetLinkProperties
        )
        self.model_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.publishModuleModel
        )
        rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.callback,
            queue_size=1
        )
        
    def publishModuleModel(self, event):
        if self.model_published:
            return
        try:
            props = self.get_link_properties(
                'beetle2::root'
            )
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(
                3.0,
                "Failed to get beetle2 properties: %s" % e
            )
            return

        if not props.success:
            rospy.logwarn_throttle(
                3.0,
                "beetle2::root not ready: %s" %
                props.status_message
            )
            return

        # 重心位置（rootリンク座標系）
        cog_offset = (
            props.com.position.x,
            props.com.position.y,
            props.com.position.z
        )

        # 慣性テンソル（Gazeboの慣性座標系）
        I = np.array([
            [props.ixx, props.ixy, props.ixz],
            [props.ixy, props.iyy, props.iyz],
            [props.ixz, props.iyz, props.izz]
        ], dtype=float)

        # 慣性座標系 -> rootリンク座標系
        q = [
            props.com.orientation.x,
            props.com.orientation.y,
            props.com.orientation.z,
            props.com.orientation.w
        ]

        R = tf.transformations.quaternion_matrix(q)[:3, :3]
        I_body = R @ I @ R.T

        if (props.mass <= 0 or
                not np.isfinite(props.mass) or
                not np.isfinite(I_body).all()):
            rospy.logerr("Invalid passive module mass or inertia")
            return

        # 受動モジュールのモデル
        msg = ModuleModel()

        msg.header.stamp = rospy.Time.now()
        msg.id = 2
        msg.is_passive = True

        msg.mass = props.mass

        # 3x3行列を9要素に変換（行優先）
        msg.inertia = [
            float(v) for v in I_body.flatten()
        ]

        # 受動モジュールなのでロータなし
        msg.rotor_origin_from_cog = []
        msg.rotor_direction = []
        msg.mf_rate = 0.0

        # 重心TFとモデルを更新
        self.cog_offset = cog_offset
        self.module_model_pub.publish(msg)

        self.model_published = True

        rospy.loginfo(
            "[PassiveModule] Published id=2 mass=%.4f "
            "cog=(%.4f, %.4f, %.4f)" %
            (
                msg.mass,
                self.cog_offset[0],
                self.cog_offset[1],
                self.cog_offset[2]
            )
        )
        
    def callback(self, msg):
        if 'beetle2' not in msg.name:
            return

        index = msg.name.index('beetle2')
        pose = msg.pose[index]

        position = (
            pose.position.x,
            pose.position.y,
            pose.position.z
        )

        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        stamp = rospy.Time.now()

        if self.last_stamp is not None:
            if stamp <= self.last_stamp:
                return

        self.br.sendTransform(
                position,
                orientation,
                stamp,
                'beetle2/root',
                'world'
            )
        self.br.sendTransform(
            tuple(self.cog_offset),
            (0.0, 0.0, 0.0, 1.0),
            stamp,
            'beetle2/cog',
            'beetle2/root'
        )

        self.last_stamp = stamp

if __name__ == '__main__':
    rospy.init_node('box_tf_broadcaster')
    BoxTFBroadcaster()
    rospy.spin()
