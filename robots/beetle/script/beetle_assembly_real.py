#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

# When executed inside robots/beetle/script, the directory contains an actual
# Python package named "beetle" which can hide generated ROS beetle.msg.
_SCRIPT_DIR = os.path.realpath(os.path.dirname(__file__))
sys.path[:] = [p for p in sys.path
               if os.path.realpath(p or os.getcwd()) != _SCRIPT_DIR]

import argparse
import math
import time

import rospy
import tf
from diagnostic_msgs.msg import KeyValue
from beetle.msg import ModuleModel

MODULE_IDS = (1, 2, 3)
ACTIVE_IDS = (1, 3)
BOX_ID = 2
LEADER_ID = 1


def parse_args():
    parser = argparse.ArgumentParser(
        description='Check / register physically assembled BEETLE 1 + Box 2 + BEETLE 3')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check-only', action='store_true',
                      help='Check TF, ModuleModel, and subscriber connections only')
    mode.add_argument('--confirm-assembled', action='store_true',
                      help='Confirm physical mechanical coupling and publish assembly flags')
    parser.add_argument('--box-mass', type=float, default=1.5,
                        help='Expected passive Box mass in kg (default: 1.5)')
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='Timeout for each check in seconds (default: 15)')
    parser.add_argument('--world-frame', default='world',
                        help='TF world frame (default: world)')
    # rospy.myargv removes ROS remapping args such as __name:=... and _foo:=...
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def check_inputs(args):
    if not math.isfinite(args.box_mass) or args.box_mass <= 0:
        raise RuntimeError('--box-mass must be a positive finite number')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise RuntimeError('--timeout must be a positive finite number')
    if not args.world_frame.strip('/'):
        raise RuntimeError('--world-frame cannot be blank')
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('/use_sim_time=true: use real-world clock for this test')


def check_tf(timeout, world):
    listener = tf.TransformListener()
    # TF listener needs a moment to receive the buffered transforms.
    rospy.sleep(0.3)
    for module_id in MODULE_IDS:
        child = 'beetle{}/cog'.format(module_id)
        rospy.loginfo('[TF] checking %s -> %s', world, child)
        listener.waitForTransform(world, child, rospy.Time(0), rospy.Duration(timeout))
        trans, quat = listener.lookupTransform(world, child, rospy.Time(0))
        if not all(math.isfinite(float(v)) for v in (*trans, *quat)):
            raise RuntimeError('TF {} has non-finite values'.format(child))
        if abs(sum(float(q) ** 2 for q in quat) - 1.0) > 0.05:
            raise RuntimeError('TF {} has invalid quaternion'.format(child))
        rospy.loginfo('[TF] %-12s  [%+.3f, %+.3f, %+.3f] OK',
                      child, trans[0], trans[1], trans[2])


def check_models(timeout, box_mass):
    for module_id in MODULE_IDS:
        topic = '/beetle{}/unified_control/module_model'.format(module_id)
        rospy.loginfo('[Model] waiting for %s', topic)
        msg = rospy.wait_for_message(topic, ModuleModel, timeout=timeout)
        if int(msg.id) != module_id:
            raise RuntimeError('{} reported id={}, expected {}'.format(
                topic, msg.id, module_id))
        if not math.isfinite(msg.mass) or msg.mass <= 0:
            raise RuntimeError('{} reported invalid mass={}'.format(topic, msg.mass))
        should_be_passive = (module_id == BOX_ID)
        if bool(msg.is_passive) != should_be_passive:
            raise RuntimeError('{} is_passive={} (expected {})'.format(
                topic, msg.is_passive, should_be_passive))
        if len(msg.inertia) != 9 or not all(math.isfinite(x) for x in msg.inertia):
            raise RuntimeError('{} has invalid 3x3 inertia'.format(topic))
        if should_be_passive:
            if abs(msg.mass - box_mass) > 0.01:
                raise RuntimeError('Box mass={} kg differs from expected {} kg'.format(
                    msg.mass, box_mass))
            if msg.rotor_origin_from_cog or msg.rotor_direction:
                raise RuntimeError('Passive Box must not advertise rotors')
        rospy.loginfo('[Model] BEETLE%d mass=%.3f kg is_passive=%s OK',
                      module_id, msg.mass, msg.is_passive)


def create_flag_publishers():
    return {
        i: rospy.Publisher('/beetle{}/assembly_flag'.format(i),
                           KeyValue, queue_size=1, latch=True)
        for i in MODULE_IDS
    }


def wait_for_subscribers(publishers, timeout):
    # Both BEETLE1 and BEETLE3 navigators should subscribe to all 3 topics.
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        missing = [
            '{} (subscribers={})'.format(i, pub.get_num_connections())
            for i, pub in publishers.items() if pub.get_num_connections() < 2
        ]
        if not missing:
            rospy.loginfo('[ROS] Assembly-flag subscribers: all 3 topics OK (>= 2)')
            return
        if time.monotonic() >= deadline:
            raise RuntimeError('Missing powered-navigator subscribers on flag '
                               'topics: {}. Verify both BEETLE1/3 bringups and '
                               'common ROS Master.'.format(', '.join(missing)))
        rospy.sleep(0.1)
    raise RuntimeError('ROS was shut down while waiting for subscribers')


def check_leader_state():
    # Navigation code fills this param once it has seen >1 assembled modules.
    for module_id in ACTIVE_IDS:
        name = '/beetle{}/assembly_leader_id'.format(module_id)
        if not rospy.has_param(name):
            rospy.logwarn('[State] %s is not available yet', name)
            continue
        leader = int(rospy.get_param(name))
        if leader != LEADER_ID:
            rospy.logerr('[State] %s=%d; expected leader ID %d. Check C++ '
                         'leader_fix_flag_ on both active modules!',
                         name, leader, LEADER_ID)
        else:
            rospy.loginfo('[State] %s=%d OK', name, leader)


def main():
    args = parse_args()
    rospy.init_node('assembly_preassembled_real', anonymous=False)
    check_inputs(args)
    rospy.loginfo('Configuration: active modules=[1,3], passive Box=2, '
                  'leader=1, expected Box mass=%.3f kg', args.box_mass)
    check_tf(args.timeout, args.world_frame.strip('/'))
    check_models(args.timeout, args.box_mass)
    pubs = create_flag_publishers()
    wait_for_subscribers(pubs, args.timeout)

    if args.check_only:
        rospy.loginfo('[PASS] Preflight checks complete; NO assembly flags sent.')
        return

    # Explicit action only after the physical connection has been verified.
    for module_id in MODULE_IDS:
        msg = KeyValue()
        msg.key = str(module_id)
        msg.value = '1'
        pubs[module_id].publish(msg)
        rospy.loginfo('[Flag] /beetle%d/assembly_flag key=%s value=%s',
                      module_id, msg.key, msg.value)
    rospy.sleep(1.0)
    check_leader_state()
    rospy.logwarn('[DONE] Assembly flags published. No motion commands sent. '
                  'module_state can remain SEPARATED until the C++ control_flag_ '
                  'condition becomes true; do NOT infer unified-flight readiness.')
    rospy.loginfo('Keep this Python running to retain latched publishers; '
                  'Ctrl+C to stop (existing flags may persist in running nodes).')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, rospy.ROSException, tf.Exception, ValueError) as exc:
        rospy.logerr('Assembly preflight FAILED: %s', exc)
        sys.exit(1)
    except KeyboardInterrupt:
        pass
