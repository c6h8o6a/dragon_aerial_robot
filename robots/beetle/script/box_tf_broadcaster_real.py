#!/usr/bin/env python3
"""Passive BEETLE/Box model and TF broadcaster (ROS1).

The executable is started ONLY by box_bringup.launch for the non-actuated Box.
It subscribes to the OptiTrack PoseStamped topic; it does NOT start a mocap
receiver and never sends motor, flight, or servo commands.

Frames: world -> beetle2/root -> beetle2/cog.
ModuleModel: id=2, is_passive=True, rotor array empty.
"""

import os
import sys
script_dir = os.path.realpath(
    os.path.dirname(__file__)
)

sys.path[:] = [
    p for p in sys.path
    if os.path.realpath(p or os.getcwd()) != script_dir
]

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from beetle.msg import ModuleModel


class BoxTFBroadcaster:
    def __init__(self):
        self.module_id = int(rospy.get_param('~module_id', 2))
        self.mocap_topic = rospy.get_param('~mocap_topic', '/beetle2/mocap/pose')
        self.module_model_topic = rospy.get_param(
            '~module_model_topic', '/beetle2/unified_control/module_model')
        self.world_frame = rospy.get_param('~world_frame', 'world').lstrip('/')
        self.root_frame = rospy.get_param('~root_frame', 'beetle2/root').lstrip('/')
        self.cog_frame = rospy.get_param('~cog_frame', 'beetle2/cog').lstrip('/')

        # Box physical parameters: never infer these from a Gazebo model.
        if rospy.get_param('~physical_parameters_verified', False) is not True:
            raise ValueError('Set ~physical_parameters_verified=true after checking mass, inertia and CoG')
        self.mass = float(self.required('mass'))
        self.inertia = self.checked_vector(self.required('inertia'), 9, 'inertia').reshape((3, 3))
        self.cog_offset = self.checked_vector(self.required('cog_offset'), 3, 'cog_offset')
        self.mocap_to_root_xyz = self.checked_vector(
            rospy.get_param('~mocap_to_root_xyz', [0, 0, 0]), 3, 'mocap_to_root_xyz')
        self.mocap_to_root_rpy = self.checked_vector(
            rospy.get_param('~mocap_to_root_rpy', [0, 0, 0]), 3, 'mocap_to_root_rpy')
        self.max_pose_age_sec = float(rospy.get_param('~max_pose_age_sec', 0.5))
        self.warn_timeout_sec = float(rospy.get_param('~warn_timeout_sec', 1.0))

        if self.module_id <= 0 or not np.isfinite(self.mass) or self.mass <= 0:
            raise ValueError('module_id must be positive; mass must be positive and finite')
        if not np.allclose(self.inertia, self.inertia.T, atol=1e-8, rtol=1e-7):
            raise ValueError('inertia tensor must be symmetric (9 row-major elements)')
        if np.linalg.eigvalsh(self.inertia).min() <= 0.0:
            raise ValueError('inertia tensor must be positive definite')
        if not np.isfinite([self.max_pose_age_sec, self.warn_timeout_sec]).all():
            raise ValueError('pose age/timeouts must be finite')
        if self.max_pose_age_sec < 0.0 or self.warn_timeout_sec <= 0.0:
            raise ValueError('max_pose_age_sec must be >= 0; warn_timeout_sec > 0')
        if not self.world_frame or not self.root_frame or not self.cog_frame:
            raise ValueError('TF frame names cannot be empty')
        if len({self.world_frame, self.root_frame, self.cog_frame}) != 3:
            raise ValueError('TF frame names must be distinct')
        if not self.mocap_topic.startswith('/') or not self.module_model_topic.startswith('/'):
            raise ValueError('Both ROS topic names must be absolute')

        self.T_mocap_root = tf.transformations.euler_matrix(*self.mocap_to_root_rpy)
        self.T_mocap_root[:3, 3] = self.mocap_to_root_xyz

        self.br = tf.TransformBroadcaster()
        self.model_pub = rospy.Publisher(
            self.module_model_topic, ModuleModel, queue_size=1, latch=True)
        self.model_published = False
        self.last_stamp = None
        self.last_receipt = None
        self.pose_sub = rospy.Subscriber(
            self.mocap_topic, PoseStamped, self.pose_callback, queue_size=1)
        self.watchdog = rospy.Timer(rospy.Duration(0.5), self.check_mocap)
        rospy.loginfo('[PassiveBox] module=%d; receiving %s', self.module_id, self.mocap_topic)
        rospy.loginfo('[PassiveBox] TF %s -> %s -> %s',
                      self.world_frame, self.root_frame, self.cog_frame)
        rospy.logwarn('[PassiveBox] No flight controller or servo is started by this node')

    @staticmethod
    def required(param):
        value = rospy.get_param('~' + param, None)
        if value is None:
            raise ValueError('~%s must be set to an actual measured/CAD value' % param)
        return value

    @staticmethod
    def checked_vector(value, count, name):
        vec = np.asarray(value, dtype=float)
        if vec.shape != (count,) or not np.isfinite(vec).all():
            raise ValueError('~%s must be %d finite numbers' % (name, count))
        return vec

    def publish_model(self):
        msg = ModuleModel()
        msg.header.stamp = rospy.Time.now()
        msg.id = self.module_id
        msg.is_passive = True
        msg.mass = self.mass
        # About Box CoG, expressed in Box root axes, row major.
        msg.inertia = [float(v) for v in self.inertia.flatten()]
        msg.rotor_origin_from_cog = []
        msg.rotor_direction = []
        msg.mf_rate = 0.0
        self.model_pub.publish(msg)
        self.model_published = True
        rospy.loginfo('[PassiveBox] published model id=%d mass=%.5f kg cog=%s',
                      self.module_id, self.mass, self.cog_offset.tolist())

    def pose_callback(self, msg):
        frame = msg.header.frame_id.lstrip('/')
        if frame and frame != self.world_frame:
            rospy.logerr_throttle(
                3.0, '[PassiveBox] pose frame "%s" is not "%s"; not publishing TF',
                frame, self.world_frame)
            return
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            # Some drivers do not stamp frames; receipt time is then used.
            stamp = rospy.Time.now()
        if self.last_stamp is not None and stamp <= self.last_stamp:
            rospy.logwarn_throttle(3.0, '[PassiveBox] ignoring non-increasing mocap timestamp')
            return
        if self.max_pose_age_sec > 0:
            age = (rospy.Time.now() - stamp).to_sec()
            if age > self.max_pose_age_sec or age < -self.max_pose_age_sec:
                rospy.logwarn_throttle(3.0, '[PassiveBox] rejecting stale/clock-skewed pose (age=%0.3f s)', age)
                return

        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
        if not np.isfinite(p).all() or not np.isfinite(q).all():
            rospy.logwarn_throttle(3.0, '[PassiveBox] rejecting non-finite pose')
            return
        qnorm = np.linalg.norm(q)
        if qnorm < 1e-9:
            rospy.logwarn_throttle(3.0, '[PassiveBox] rejecting zero quaternion')
            return
        q /= qnorm

        T_world_mocap = tf.transformations.quaternion_matrix(q)
        T_world_mocap[:3, 3] = p
        T_world_root = T_world_mocap.dot(self.T_mocap_root)
        root_pos = tuple(float(x) for x in T_world_root[:3, 3])
        root_quat = tuple(float(x) for x in tf.transformations.quaternion_from_matrix(T_world_root))

        self.br.sendTransform(root_pos, root_quat, stamp,
                              self.root_frame, self.world_frame)
        self.br.sendTransform(tuple(float(x) for x in self.cog_offset),
                              (0., 0., 0., 1.), stamp,
                              self.cog_frame, self.root_frame)
        self.last_stamp = stamp
        self.last_receipt = rospy.Time.now()
        if not self.model_published:
            # Latched model is advertised only after a valid measured pose.
            self.publish_model()

    def check_mocap(self, _event):
        if self.last_receipt is None:
            rospy.logwarn_throttle(5.0, '[PassiveBox] waiting for %s (no usable pose)', self.mocap_topic)
        elif (rospy.Time.now() - self.last_receipt).to_sec() > self.warn_timeout_sec:
            rospy.logerr_throttle(2.0, '[PassiveBox] Mocap timed out; TF is no longer being updated')


if __name__ == '__main__':
    rospy.init_node('box_tf_broadcaster_real')
    try:
        BoxTFBroadcaster()
    except (ValueError, TypeError, KeyError, rospy.ROSException) as exc:
        rospy.logfatal('[PassiveBox] bad model/config: %s', exc)
        raise
    rospy.spin()
