#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import re
import sys
import time

import rospy
import yaml
from std_msgs.msg import String

try:
    import tf
except ImportError:
    tf = None

try:
    unicode
except NameError:
    unicode = str

PY2 = sys.version_info[0] == 2


def to_text(value):
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    if isinstance(value, unicode):
        return value
    return unicode(value)


def ros_string(value):
    value = to_text(value)
    if PY2:
        return value.encode('utf-8')
    return value


def param_bool(value):
    if isinstance(value, bool):
        return value
    return to_text(value).strip().lower() in ('1', 'true', 'yes', 'on')


def load_waypoints(path):
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle) or {}
    return data.get('waypoints', {})


def target_payload(name):
    name = to_text(name).strip()
    if name.startswith('window_') and len(name) >= 8:
        return name[-1].upper()
    if name.startswith('lab_window_'):
        match = re.search(r'[1-4]', name)
        return match.group(0) if match else ''
    return ''


def target_kind(name):
    if to_text(name).startswith('window_'):
        return 'window'
    if to_text(name).startswith('lab_window_'):
        return 'lab'
    return 'other'


class AutoArrivalDetector(object):
    def __init__(self):
        if tf is None:
            raise RuntimeError('tf is not available; cannot auto-detect arrival')

        self.waypoints_path = rospy.get_param('~waypoints')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.arrival_topic = rospy.get_param('~arrival_topic', '/smart_pharmacy_patrol/manual_arrival')
        self.status_topic = rospy.get_param('~status_topic', '/smart_pharmacy_patrol/status')
        self.detector_status_topic = rospy.get_param(
            '~detector_status_topic',
            '/smart_pharmacy_patrol/auto_arrival_status',
        )
        self.pickup_radius = float(rospy.get_param('~pickup_radius', 0.32))
        self.lab_radius = float(rospy.get_param('~lab_radius', 0.32))
        self.dwell_sec = float(rospy.get_param('~dwell_sec', 1.0))
        self.cooldown_sec = float(rospy.get_param('~cooldown_sec', 12.0))
        self.max_speed = float(rospy.get_param('~max_speed', 0.05))
        self.require_stop = param_bool(rospy.get_param('~require_stop', True))
        self.require_board1 = param_bool(rospy.get_param('~require_board1', True))
        self.require_collection_for_lab = param_bool(rospy.get_param('~require_collection_for_lab', False))
        self.repeat_targets = param_bool(rospy.get_param('~repeat_targets', False))

        active_targets = to_text(rospy.get_param(
            '~active_targets',
            'window_A,window_B,window_C,lab_window_1,lab_window_2,lab_window_3,lab_window_4',
        ))
        requested = [item.strip() for item in re.split(r'[,;| ]+', active_targets) if item.strip()]

        all_waypoints = load_waypoints(self.waypoints_path)
        self.targets = {}
        for name in requested:
            if name not in all_waypoints:
                rospy.logwarn('[auto_arrival_detector] waypoint not found: %s', name)
                continue
            payload = target_payload(name)
            if not payload:
                rospy.logwarn('[auto_arrival_detector] unsupported target: %s', name)
                continue
            point = all_waypoints[name]
            self.targets[name] = {
                'x': float(point['x']),
                'y': float(point['y']),
                'kind': target_kind(name),
                'payload': payload,
            }

        self.arrival_pub = rospy.Publisher(self.arrival_topic, String, queue_size=10)
        self.detector_status_pub = rospy.Publisher(self.detector_status_topic, String, queue_size=10)
        rospy.Subscriber(self.status_topic, String, self.on_status, queue_size=30)

        self.tf_listener = tf.TransformListener()
        self.board1_ready = False
        self.collection_count = 0
        self.inside_since = {}
        self.triggered_at = {}
        self.previous_pose = None
        self.previous_pose_time = None

        rospy.loginfo(
            '[auto_arrival_detector] ready targets=%s waypoints=%s radius pickup=%.2f lab=%.2f',
            ','.join(sorted(self.targets.keys())),
            self.waypoints_path,
            self.pickup_radius,
            self.lab_radius,
        )

    def on_status(self, msg):
        text = to_text(msg.data).strip()
        if text.startswith('vision_board_1:'):
            self.board1_ready = True
        elif text.startswith('collected:'):
            self.collection_count += 1
        elif text.startswith('cycle_complete'):
            self.collection_count = 0
            if not self.repeat_targets:
                self.triggered_at = {}

    def lookup_pose(self):
        try:
            trans, _ = self.tf_listener.lookupTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
            )
        except Exception:
            return None
        return float(trans[0]), float(trans[1])

    def estimate_speed(self, pose, now):
        if self.previous_pose is None or self.previous_pose_time is None:
            self.previous_pose = pose
            self.previous_pose_time = now
            return 0.0
        dt = max(1e-3, now - self.previous_pose_time)
        speed = math.hypot(pose[0] - self.previous_pose[0], pose[1] - self.previous_pose[1]) / dt
        self.previous_pose = pose
        self.previous_pose_time = now
        return speed

    def radius_for(self, target):
        if target['kind'] == 'lab':
            return self.lab_radius
        return self.pickup_radius

    def gate_allows(self, target):
        if not self.require_board1:
            return True
        if not self.board1_ready:
            return False
        if target['kind'] == 'lab' and self.require_collection_for_lab and self.collection_count <= 0:
            return False
        return True

    def maybe_trigger(self, name, target, now):
        last = self.triggered_at.get(name, 0.0)
        if last and not self.repeat_targets:
            return
        if last and now - last < self.cooldown_sec:
            return

        self.triggered_at[name] = now
        payload = target['payload']
        self.arrival_pub.publish(String(data=ros_string(payload)))
        line = 'auto_arrived:%s;payload=%s' % (name, payload)
        self.detector_status_pub.publish(String(data=ros_string(line)))
        rospy.loginfo('[auto_arrival_detector] %s', line)

    def run(self):
        rate_hz = float(rospy.get_param('~rate_hz', 10.0))
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            now = time.time()
            pose = self.lookup_pose()
            if pose is None:
                rospy.logwarn_throttle(5.0, '[auto_arrival_detector] waiting for TF %s -> %s', self.map_frame, self.base_frame)
                rate.sleep()
                continue

            speed = self.estimate_speed(pose, now)
            nearest_name = ''
            nearest_dist = 999.0
            for name, target in self.targets.items():
                dist = math.hypot(pose[0] - target['x'], pose[1] - target['y'])
                if dist < nearest_dist:
                    nearest_name = name
                    nearest_dist = dist

                inside = dist <= self.radius_for(target)
                stopped_enough = (not self.require_stop) or speed <= self.max_speed
                allowed = self.gate_allows(target)
                if inside and stopped_enough and allowed:
                    if name not in self.inside_since:
                        self.inside_since[name] = now
                    elif now - self.inside_since[name] >= self.dwell_sec:
                        self.maybe_trigger(name, target, now)
                else:
                    self.inside_since.pop(name, None)

            self.detector_status_pub.publish(String(data=ros_string(
                'pose=%.3f,%.3f;speed=%.3f;nearest=%s;dist=%.3f;board1_ready=%s;collections=%d'
                % (pose[0], pose[1], speed, nearest_name, nearest_dist, self.board1_ready, self.collection_count)
            )))
            rate.sleep()


def main():
    rospy.init_node('auto_arrival_detector')
    AutoArrivalDetector().run()


if __name__ == '__main__':
    main()
