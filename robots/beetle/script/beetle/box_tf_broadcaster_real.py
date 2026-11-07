#!/usr/bin/env python3
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from beetle.msg import ModuleModel


class BoxTFBroadcaster:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.last_stamp = None
        self.world_frame = rospy.get_param('~world_frame', 'world').lstrip('/')
        self.mocap_topic = rospy.get_param('~mocap_topic', '/vrpn_client_node/box/pose')
        self.cog_offset = np.asarray(rospy.get_param('~cog_offset'), dtype=float)
        self.mass = float(rospy.get_param('~mass'))
        self.inertia = np.asarray(rospy.get_param('~inertia'), dtype=float)
        self.mocap_to_root_xyz = np.asarray(
            rospy.get_param('~mocap_to_root_xyz', [0.0, 0.0, 0.0]), dtype=float)
        self.mocap_to_root_rpy = np.asarray(
            rospy.get_param('~mocap_to_root_rpy', [0.0, 0.0, 0.0]), dtype=float)

        if (self.cog_offset.shape != (3) or
                self.mocap_to_root_xyz.shape != (3) or
                self.mocap_to_root_rpy.shape != (3) or
                self.inertia.shape != (9,)):
            raise ValueError('cog_offset/mocap_to_root_xyz/rpy require 3 values; inertia requires 9')
        self.inertia = self.inertia.reshape((3, 3))
        if (not np.isfinite(self.mass) or self.mass <= 0 or
                not np.isfinite(self.cog_offset).all() or
                not np.isfinite(self.inertia).all() or
                not np.isfinite(self.mocap_to_root_xyz).all() or
                not np.isfinite(self.mocap_to_root_rpy).all()):
            raise ValueError('Invalid mass, inertia, or coordinate offsets')
        if not np.allclose(self.inertia, self.inertia.T, atol=1e-8):
            raise ValueError('Inertia tensor must be symmetric')
        if np.linalg.eigvalsh(self.inertia).min() <= 0:
            raise ValueError('Inertia tensor must be positive definite')

        self.T_mocap_root = tf.transformations.euler_matrix(*self.mocap_to_root_rpy)
        self.T_mocap_root[:3, 3] = self.mocap_to_root_xyz

        self.module_model_pub = rospy.Publisher(
            '/beetle2/unified_control/module_model', ModuleModel,
            queue_size=1, latch=True)
        self.publish_module_model()
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.callback, queue_size=1)
        rospy.loginfo('Listening to Box MoCap pose: %s', self.mocap_topic)

    def publish_module_model(self):
        msg = ModuleModel()
        msg.header.stamp = rospy.Time.now()
        msg.id = 2
        msg.is_passive = True
        msg.mass = self.mass
        # Tensor about Box CoG, expressed in beetle2/root axes (row major).
        msg.inertia = [float(x) for x in self.inertia.flatten()]
        msg.rotor_origin_from_cog = []
        msg.rotor_direction = []
        msg.mf_rate = 0.0
        self.module_model_pub.publish(msg)
        rospy.loginfo('[PassiveBox] model id=2 mass=%.4f kg, CoG=%s',
                      self.mass, self.cog_offset.tolist())

    def callback(self, msg):
        frame = msg.header.frame_id.lstrip('/')
        if frame and frame != self.world_frame:
            rospy.logwarn_throttle(
                3.0, 'MoCap pose is in frame "%s", not "%s". '
                'Transform into world frame before using this node.',
                frame, self.world_frame)
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return

        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
        if not np.isfinite(p).all() or not np.isfinite(q).all():
            rospy.logwarn_throttle(3.0, 'Box MoCap pose contains NaN or Inf')
            return
        q_norm = np.linalg.norm(q)
        if q_norm < 1e-9:
            rospy.logwarn_throttle(3.0, 'Invalid Box quaternion')
            return
        q /= q_norm

        T_world_mocap = tf.transformations.quaternion_matrix(q)
        T_world_mocap[:3, 3] = p
        T_world_root = T_world_mocap @ self.T_mocap_root
        root_position = T_world_root[:3, 3].tolist()
        root_orientation = tf.transformations.quaternion_from_matrix(T_world_root).tolist()

        self.br.sendTransform(root_position, root_orientation, stamp,
                              'beetle2/root', self.world_frame)
        self.br.sendTransform(self.cog_offset.tolist(), (0.0, 0.0, 0.0, 1.0),
                              stamp, 'beetle2/cog', 'beetle2/root')
        self.last_stamp = stamp


if __name__ == '__main__':
    rospy.init_node('box_tf_broadcaster')
    try:
        BoxTFBroadcaster()
    except (ValueError, KeyError, rospy.ROSException) as exc:
        rospy.logfatal('Box TF broadcaster configuration error: %s', exc)
        raise
    rospy.spin()
