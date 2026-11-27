#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 1: estimate added payload mass, location, and box CoM shift.

This is a read-only diagnostic node. It never changes controller commands.

The formation controller publishes model_error_estimate as:
  data[0] = unmodeled mass dm = -F_ext_z/g                   [kg]
  data[1] = formation CoM shift dx = tau_ext_y/(M_true*g)   [m]
  data[2] = formation CoM shift dy = -tau_ext_x/(M_true*g)  [m]
  data[6] = modeled formation mass M_model                  [kg]

Those formulas assume a near-level, quiet hover and gravity-dominated residuals.
The unknown payload position in the *modeled formation CoG* frame is:
  r_payload/G_model_xy = (M_model + dm)/dm * [dx, dy].

We recover the modeled formation CoG from the nominal mass-weighted module
CoG TF positions, then subtract the box root position. The SDF supplied by the
user puts the nominal 1.5-kg box CoG at box/root, so:
  r_payload/box_root_xy = r_payload/G_model_xy - r_box_root/G_model_xy
  delta_box_CoG_xy = dm/(box_model_mass + dm) * r_payload/box_root_xy

All published XY values are in metres, approximately along the leader virtual
CoG axes. Z position is NOT inferred from level-hover gravity moments.
"""

import math
from tf.transformations import euler_from_quaternion
import rospy
import rostopic
import tf2_ros
from std_msgs.msg import Float64, Float64MultiArray

PID_TOPIC = '/assemble/debug/pose/pid'
OBSERVER_TOPIC = '/beetle1/unified_control/model_error_estimate'


def vec3(transform):
    t = transform.transform.translation
    return (t.x, t.y, t.z)


class MassEstimationNode:
    def __init__(self):
        self.model_mass = float(rospy.get_param('~model_mass', 8.882586))
        self.gravity = float(rospy.get_param('~gravity', 9.797))
        self.i_baseline = float(rospy.get_param('~i_baseline', 0.0))
        self.box_model_mass = float(rospy.get_param('~box_model_mass', 1.5))
        self.beetle1_mass = float(rospy.get_param('~beetle1_mass', 3.6912934))
        self.beetle3_mass = float(rospy.get_param('~beetle3_mass', 3.6912934))
        self.min_payload_mass = float(rospy.get_param('~min_payload_mass', 0.20))
        self.tf_timeout = float(rospy.get_param('~tf_timeout', 0.20))
        self.leader_frame = rospy.get_param('~leader_frame', 'beetle1/cog')
        self.box_cog_frame = rospy.get_param('~box_cog_frame', 'beetle2/cog')
        self.box_root_frame = rospy.get_param('~box_root_frame', 'beetle2/root')
        self.follower_frame = rospy.get_param('~follower_frame', 'beetle3/cog')
        self.require_torque_enabled = bool(rospy.get_param('~require_torque_enabled', True))
        self.torque_param = rospy.get_param(
            '~torque_param',
            '/beetle1/controller/formation_observer/enable_torque_observer')
        self.tau_x_baseline = float(
            rospy.get_param('~tau_x_baseline', 0.0))
        self.tau_y_baseline = float(
            rospy.get_param('~tau_y_baseline', 0.0))

        if self.model_mass <= 0.0 or self.gravity <= 0.0 or self.box_model_mass <= 0.0:
            raise ValueError('model_mass, gravity, box_model_mass must be positive')
        if min(self.beetle1_mass, self.beetle3_mass) <= 0.0:
            raise ValueError('beetle1_mass and beetle3_mass must be positive')

        self.module_model_mass = (
            self.beetle1_mass + self.box_model_mass + self.beetle3_mass)
        if abs(self.module_model_mass - self.model_mass) > 0.01:
            rospy.logwarn('Mass mismatch: model_mass=%.6f vs sum(module masses)=%.6f. '
                          'Check the nominal module masses before interpreting position.',
                          self.model_mass, self.module_model_mass)

        # Original estimate topics: preserved.
        self.i_term_pub = rospy.Publisher('/mass_estimation/i_term_z', Float64, queue_size=20)
        self.i_mass_pub = rospy.Publisher('/mass_estimation/i_delta_mass', Float64, queue_size=20)
        self.i_corrected_pub = rospy.Publisher(
            '/mass_estimation/i_delta_mass_corrected', Float64, queue_size=20)
        self.observer_mass_pub = rospy.Publisher(
            '/mass_estimation/observer_delta_mass', Float64, queue_size=20)

        # XY pair is Float64MultiArray([x_m, y_m]). These are planar estimates.
        self.payload_formation_pub = rospy.Publisher(
            '/mass_estimation/weight_from_formation_xy', Float64MultiArray, queue_size=20)
        self.box_root_formation_pub = rospy.Publisher(
            '/mass_estimation/box_root_from_formation_xy', Float64MultiArray, queue_size=20)
        self.payload_box_pub = rospy.Publisher(
            '/mass_estimation/weight_from_box_xy', Float64MultiArray, queue_size=20)
        self.box_cog_shift_pub = rospy.Publisher(
            '/mass_estimation/box_cog_shift_xy', Float64MultiArray, queue_size=20)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Retain runtime resolution of the project's custom PID message.
        pid_class, _, _ = rostopic.get_topic_class(PID_TOPIC, blocking=True)
        observer_class, _, _ = rostopic.get_topic_class(OBSERVER_TOPIC, blocking=True)
        if pid_class is None or observer_class is None:
            raise RuntimeError('Required ROS topics are not available')

        rospy.Subscriber(PID_TOPIC, pid_class, self.on_pid, queue_size=100)
        rospy.Subscriber(OBSERVER_TOPIC, observer_class, self.on_observer, queue_size=100)
        rospy.loginfo('Mass + payload position estimator: model_mass=%.6f kg, '
                      'box_model_mass=%.3f kg, i_baseline=%.6f',
                      self.model_mass, self.box_model_mass, self.i_baseline)
        rospy.loginfo('Position reference frame: %s (nominal formation CoG reconstructed '
                      'from module CoG TF and nominal masses)', self.leader_frame)

    def on_pid(self, msg):
        try:
            iz = float(msg.z.i_term[0])
        except (AttributeError, IndexError, TypeError, ValueError):
            rospy.logwarn_throttle(5.0, 'Could not read msg.z.i_term[0] from PID topic')
            return
        if not math.isfinite(iz):
            return

        self.i_term_pub.publish(Float64(data=iz))
        self.i_mass_pub.publish(Float64(data=self.model_mass * iz / self.gravity))
        self.i_corrected_pub.publish(
            Float64(data=self.model_mass * (iz - self.i_baseline) / self.gravity))

    def pos(self, source_frame):
        t = self.tf_buffer.lookup_transform(
            self.leader_frame, source_frame,
            rospy.Time(0), rospy.Duration(self.tf_timeout))
        return vec3(t)

    def box_yaw(self):
        t = self.tf_buffer.lookup_transform(
            self.leader_frame,
            self.box_root_frame,
            rospy.Time(0),
            rospy.Duration(self.tf_timeout))
        
        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        
        return yaw

    @staticmethod
    def publish_xy(pub, x, y):
        pub.publish(Float64MultiArray(data=[x, y]))

    def on_observer(self, msg):
        try:
            dm = float(msg.data[0])
            dx_form = float(msg.data[1])
            dy_form = float(msg.data[2])
            reported_model_mass = float(msg.data[6])
        except (AttributeError, IndexError, TypeError, ValueError):
            rospy.logwarn_throttle(5.0, 'Unexpected model_error_estimate layout')
            return
        if not all(math.isfinite(x) for x in (dm, dx_form, dy_form, reported_model_mass)):
            return
        self.observer_mass_pub.publish(Float64(data=dm))
        if abs(reported_model_mass - self.model_mass) > 0.01:
            rospy.logwarn_throttle(
                5.0, 'Model-mass mismatch: node=%.5f, controller=%.5f',
                self.model_mass, reported_model_mass)
            return

        # A disabled torque observer normally leaves x/y at zero, which must
        # NOT be interpreted as a valid inferred payload position.
        if self.require_torque_enabled and not rospy.get_param(self.torque_param, False):
            rospy.logwarn_throttle(
                5.0, 'Torque observer is not confirmed enabled (%s). '
                'Set it in the controller config and restart the controller.',
                self.torque_param)
            return

        if abs(dm) < self.min_payload_mass:
            rospy.logwarn_throttle(
                5.0, 'Unmodeled mass %.3f kg is too close to zero for payload-position '
                'division (threshold %.3f kg).', dm, self.min_payload_mass)
            return
        if self.box_model_mass + dm <= 0.0 or reported_model_mass + dm <= 0.0:
            return

        # Formation delta CoM from gravity residual -> added mass location
        # relative to the MODELED formation CoG (not the true shifted CoG).
        total_true_mass = reported_model_mass + dm
        # 結合体オブザーバから推定された外モーメント
        tau_x = -dy_form * total_true_mass * self.gravity
        tau_y =  dx_form * total_true_mass * self.gravity

        # おもりを搭載していない状態での残差を除去
        tau_x_corrected = tau_x - self.tau_x_baseline
        tau_y_corrected = tau_y - self.tau_y_baseline

        # 追加したおもりの位置を推定
        weight_G_x = tau_y_corrected / (dm * self.gravity)
        weight_G_y = -tau_x_corrected / (dm * self.gravity)

        try:
            # All positions are expressed in leader virtual CoG axes.
            # The leader itself is the coordinate origin.
            box_cog = self.pos(self.box_cog_frame)
            beetle3_cog = self.pos(self.follower_frame)
            box_root = self.pos(self.box_root_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5.0, 'Unable to compute box-relative CoM: %s', str(exc))
            return

        # Original assembly CoG (nominal masses, WITHOUT 1-kg payload).
        G_model_x = (self.box_model_mass * box_cog[0]
                     + self.beetle3_mass * beetle3_cog[0]) / self.module_model_mass
        G_model_y = (self.box_model_mass * box_cog[1]
                     + self.beetle3_mass * beetle3_cog[1]) / self.module_model_mass

        # Actual box root position in nominal assembly CoG coordinates.
        box_G_x = box_root[0] - G_model_x
        box_G_y = box_root[1] - G_model_y

        # Payload location in BOX ROOT coordinates; estimated from gravity moments.
        # 箱原点を基準としたリーダー座標軸上の位置
        dx = weight_G_x - box_G_x
        dy = weight_G_y - box_G_y

        # 箱とリーダーの相対Yaw角
        try:
            yaw = self.box_yaw()
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        
        c = math.cos(yaw)
        s = math.sin(yaw)

        # リーダー座標軸から箱座標軸へ回転
        weight_box_x =  c * dx + s * dy
        weight_box_y = -s * dx + c * dy

        # SDF assumption: nominal 1.5 kg box CoG is at box/root (inertial pose=0).
        fraction = dm / (self.box_model_mass + dm)
        box_shift_x = fraction * weight_box_x
        box_shift_y = fraction * weight_box_y

        if not all(math.isfinite(x) for x in (
                weight_box_x, weight_box_y, box_shift_x, box_shift_y)):
            return

        self.publish_xy(self.payload_formation_pub, weight_G_x, weight_G_y)
        self.publish_xy(self.box_root_formation_pub, box_G_x, box_G_y)
        self.publish_xy(self.payload_box_pub, weight_box_x, weight_box_y)
        self.publish_xy(self.box_cog_shift_pub, box_shift_x, box_shift_y)

        rospy.loginfo_throttle(
            2.0, 'dm=%+.3f kg | weight/G_model=(%+.2f,%+.2f) mm | '
            'box_root/G_model=(%+.2f,%+.2f) mm | '
            'weight/box=(%+.2f,%+.2f) mm | '
            'delta_box_CoG=(%+.2f,%+.2f) mm',
            dm, weight_G_x*1000, weight_G_y*1000,
            box_G_x*1000, box_G_y*1000,
            weight_box_x*1000, weight_box_y*1000,
            box_shift_x*1000, box_shift_y*1000)


if __name__ == '__main__':
    rospy.init_node('mass_estimation_i_vs_observer')
    MassEstimationNode()
    rospy.spin()
