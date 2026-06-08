#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import json
import math
import socket
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    unicode
except NameError:
    unicode = str

PY2 = sys.version_info[0] == 2

try:
    import tf
except ImportError:
    tf = None


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


def to_bytes(value):
    value = to_text(value)
    return value.encode('utf-8')


def waypoint_to_task(waypoint):
    waypoint = to_text(waypoint).strip()
    if waypoint.startswith('window_') and len(waypoint) >= 8:
        return waypoint[-1].upper()
    if waypoint.startswith('lab_window_'):
        return waypoint.replace('lab_window_', '')
    return waypoint


def compact_windows(value):
    value = to_text(value).upper()
    windows = []
    for char in value:
        if char in 'ABC' and char not in windows:
            windows.append(char)
    return ''.join(windows)


def parse_status_fields(text):
    text = to_text(text).strip()
    if ':' in text:
        text = text.split(':', 1)[1]
    fields = {}
    for part in text.split(';'):
        if '=' not in part:
            continue
        key, value = part.split('=', 1)
        fields[key.strip().lower()] = value.strip()
    return fields


def cv1_from_status(text):
    fields = parse_status_fields(text)
    if 'tasks' in fields:
        return fields['tasks'].strip()

    selected_slot = fields.get('selected_slot', fields.get('slot', '')).strip()
    if 'qr' in fields:
        if selected_slot:
            return '%s:%s' % (selected_slot, fields['qr'].strip())
        return fields['qr'].strip()

    windows = compact_windows(fields.get('windows', ''))
    target_lab = fields.get('target_lab', fields.get('lab', '')).strip()
    if windows and target_lab:
        return '%s:%s' % (windows, target_lab)
    if windows:
        return windows
    return ''


def cv2_from_status(text):
    fields = parse_status_fields(text)
    status = fields.get('status', '').strip().lower()
    wait = fields.get('wait', fields.get('wait_time', '')).strip()
    if status == 'free':
        return 'FREE'
    if status == 'busy':
        return 'BUSY:%s' % (wait or '5')
    return ''


class JudgeBridge(object):
    def __init__(self):
        self.host = to_text(rospy.get_param('~judge_host', '192.168.12.248')).strip()
        self.port = int(rospy.get_param('~judge_port', 8888))
        self.source_ip = to_text(rospy.get_param('~source_ip', '')).strip()
        self.send_hz = float(rospy.get_param('~send_hz', 5.0))
        self.reconnect_delay = float(rospy.get_param('~reconnect_delay', 1.0))
        self.connect_timeout = float(rospy.get_param('~connect_timeout', 3.0))
        self.map_frame = to_text(rospy.get_param('~map_frame', 'map')).strip()
        self.base_frame = to_text(rospy.get_param('~base_frame', 'base_footprint')).strip()
        self.speed_absolute = param_bool(rospy.get_param('~speed_absolute', True))
        self.include_stamp = param_bool(rospy.get_param('~include_stamp', False))

        self.task = to_text(rospy.get_param('~initial_task', 'start')).strip()
        self.cv1 = to_text(rospy.get_param('~initial_cv1', '')).strip()
        self.cv2 = to_text(rospy.get_param('~initial_cv2', '')).strip()
        self.odom_xy = [0.0, 0.0]
        self.cmd_speed = 0.0
        self.odom_speed = 0.0
        self.last_cmd_time = 0.0
        self.last_odom_time = 0.0

        self.sock = None
        self.lock = threading.RLock()
        self.last_connected = False
        self.pending_crash = False

        self.status_pub = rospy.Publisher('/judge_bridge/status', String, queue_size=10)
        self.payload_pub = rospy.Publisher('/judge_bridge/payload', String, queue_size=10)

        rospy.Subscriber(rospy.get_param('~cmd_vel_topic', '/cmd_vel'), Twist, self.on_cmd_vel, queue_size=20)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/odometry/filtered'), Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber(rospy.get_param('~fallback_odom_topic', '/odom'), Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber(rospy.get_param('~mission_status_topic', '/smart_pharmacy_patrol/status'), String, self.on_mission_status, queue_size=20)
        rospy.Subscriber(rospy.get_param('~task_topic', '/judge/task'), String, self.on_task, queue_size=10)
        rospy.Subscriber(rospy.get_param('~cv1_topic', '/judge/cv1'), String, self.on_cv1, queue_size=10)
        rospy.Subscriber(rospy.get_param('~cv2_topic', '/judge/cv2'), String, self.on_cv2, queue_size=10)
        rospy.Subscriber(rospy.get_param('~crash_topic', '/judge/crash'), String, self.on_crash, queue_size=10)

        self.tf_listener = tf.TransformListener() if tf is not None else None

    def publish_status(self, text):
        self.status_pub.publish(String(data=ros_string(text)))

    def publish_payload(self, text):
        self.payload_pub.publish(String(data=ros_string(text)))

    def on_cmd_vel(self, msg):
        with self.lock:
            self.cmd_speed = float(msg.linear.x)
            self.last_cmd_time = time.time()

    def on_odom(self, msg):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        with self.lock:
            self.odom_xy = [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            ]
            self.odom_speed = math.sqrt(vx * vx + vy * vy)
            self.last_odom_time = time.time()

    def on_task(self, msg):
        with self.lock:
            self.task = to_text(msg.data).strip()

    def on_cv1(self, msg):
        with self.lock:
            self.cv1 = to_text(msg.data).strip()

    def on_cv2(self, msg):
        with self.lock:
            self.cv2 = to_text(msg.data).strip()

    def on_crash(self, msg):
        text = to_text(msg.data).strip().lower()
        with self.lock:
            self.pending_crash = text not in ('', '0', 'false', 'no')

    def on_mission_status(self, msg):
        text = to_text(msg.data).strip()
        next_task = None
        next_cv1 = None
        next_cv2 = None

        if text.startswith('navigating:'):
            next_task = waypoint_to_task(text.split(':', 1)[1])
        elif text.startswith('navigation_succeeded:'):
            next_task = waypoint_to_task(text.split(':', 1)[1])
        elif text.startswith('recognizing:board_1'):
            next_task = 'board_1'
        elif text.startswith('recognizing:board_2'):
            next_task = 'board_2'
        elif text.startswith('collected:'):
            next_task = text.split(':', 1)[1].split(';', 1)[0].strip().upper()
        elif text.startswith('dropped:lab='):
            next_task = text.split('=', 1)[1].split(';', 1)[0].strip()
        elif text.startswith('cycle_complete'):
            next_task = 'start'

        if text.startswith('vision_board_1:'):
            next_cv1 = cv1_from_status(text)
        elif text.startswith('vision_board_2:'):
            next_cv2 = cv2_from_status(text)

        with self.lock:
            if next_task:
                self.task = next_task
            if next_cv1:
                self.cv1 = next_cv1
            if next_cv2:
                self.cv2 = next_cv2

    def update_tf_pose(self):
        if self.tf_listener is None:
            return
        try:
            trans, _ = self.tf_listener.lookupTransform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
            )
        except Exception:
            return

        with self.lock:
            self.odom_xy = [float(trans[0]), float(trans[1])]
            self.last_odom_time = time.time()

    def current_speed(self):
        now = time.time()
        with self.lock:
            if now - self.last_cmd_time < 1.0:
                speed = self.cmd_speed
            else:
                speed = self.odom_speed
            if self.speed_absolute:
                speed = abs(speed)
            return speed

    def snapshot(self):
        self.update_tf_pose()
        with self.lock:
            payload = {
                'task': self.task,
                'speed': round(self.current_speed(), 3),
                'odom': [round(self.odom_xy[0], 3), round(self.odom_xy[1], 3)],
                'CV1': self.cv1,
                'CV2': self.cv2,
            }
            if self.include_stamp:
                payload['stamp'] = round(time.time(), 3)
        return payload

    def close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.last_connected = False

    def ensure_connected(self):
        if self.sock is not None:
            return
        if self.source_ip:
            self.sock = socket.create_connection(
                (self.host, self.port),
                self.connect_timeout,
                source_address=(self.source_ip, 0),
            )
        else:
            self.sock = socket.create_connection((self.host, self.port), self.connect_timeout)
        self.sock.settimeout(self.connect_timeout)
        self.last_connected = True
        message = 'connected %s:%d' % (self.host, self.port)
        if self.source_ip:
            message += ' from %s' % self.source_ip
        rospy.loginfo('[judge_bridge] ' + message)
        self.publish_status(message)

    def send_once(self):
        payload = self.snapshot()
        encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.ensure_connected()

        with self.lock:
            send_crash = self.pending_crash
            self.pending_crash = False
        if send_crash:
            self.sock.sendall(b'CRASH\n')

        self.sock.sendall(to_bytes(encoded) + b'\n')
        self.publish_payload(encoded)
        return payload

    def run(self):
        rate = rospy.Rate(self.send_hz)
        rospy.loginfo('[judge_bridge] sending to %s:%d at %.2f Hz', self.host, self.port, self.send_hz)
        while not rospy.is_shutdown():
            try:
                payload = self.send_once()
                rospy.loginfo_throttle(5.0, '[judge_bridge] sent %s' % payload)
            except Exception as exc:
                if self.last_connected:
                    self.publish_status('disconnected: %s' % exc)
                rospy.logwarn_throttle(3.0, '[judge_bridge] send/connect failed: %s' % exc)
                self.close_socket()
                rospy.sleep(self.reconnect_delay)
            rate.sleep()
        self.close_socket()


def main():
    rospy.init_node('judge_bridge')
    JudgeBridge().run()


if __name__ == '__main__':
    main()
