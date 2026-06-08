#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import re
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

try:
    unicode
except NameError:
    unicode = str

PY2 = sys.version_info[0] == 2

WINDOW_WAYPOINTS = {
    'A': 'window_A',
    'B': 'window_B',
    'C': 'window_C',
}

GOAL_STATUS_NAMES = {
    GoalStatus.PENDING: 'PENDING',
    GoalStatus.ACTIVE: 'ACTIVE',
    GoalStatus.PREEMPTED: 'PREEMPTED',
    GoalStatus.SUCCEEDED: 'SUCCEEDED',
    GoalStatus.ABORTED: 'ABORTED',
    GoalStatus.REJECTED: 'REJECTED',
    GoalStatus.PREEMPTING: 'PREEMPTING',
    GoalStatus.RECALLING: 'RECALLING',
    GoalStatus.RECALLED: 'RECALLED',
    GoalStatus.LOST: 'LOST',
}


def to_text(value):
    if isinstance(value, unicode):
        return value
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    try:
        return unicode(value, 'utf-8', errors='replace')
    except TypeError:
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


def log_text(prefix, text):
    rospy.loginfo('%s%s' % (prefix, to_text(text)))


def load_waypoints(path):
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle) or {}
    return data.get('waypoints', {})


def make_goal(point):
    theta = float(point['theta'])
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(point['x'])
    goal.target_pose.pose.position.y = float(point['y'])
    goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)
    return goal


def parse_status_fields(text):
    text = to_text(text).strip()
    if ':' in text and text.split(':', 1)[0].lower().startswith('vision_board'):
        text = text.split(':', 1)[1]
    fields = {}
    for part in re.split(r';+', text):
        if '=' not in part:
            continue
        key, value = part.split('=', 1)
        fields[key.strip().lower()] = value.strip()
    return fields


def compact_windows(value):
    value = to_text(value).upper()
    windows = []
    for char in value:
        if char in WINDOW_WAYPOINTS and char not in windows:
            windows.append(char)
    return windows


def clean_lab(value, default_lab='1'):
    value = to_text(value).strip()
    match = re.search(r'[1-4]', value)
    return match.group(0) if match else default_lab


def task_from_slot_qr(slot, qr, default_lab='1'):
    lab = clean_lab(slot, default_lab)
    qr = to_text(qr).strip().upper()
    windows = compact_windows(qr)
    if not windows:
        windows = ['A']
    return {
        'slot': lab,
        'target_lab': lab,
        'qr': qr,
        'windows': windows,
    }


def task_from_windows_lab(windows_text, lab_text):
    lab = clean_lab(lab_text, '1')
    qr = ''.join(compact_windows(windows_text)) or to_text(windows_text).strip().upper()
    windows = compact_windows(windows_text) or ['A']
    return {
        'slot': lab,
        'target_lab': lab,
        'qr': qr,
        'windows': windows,
    }


def parse_task_list(value, default_lab='1'):
    text = to_text(value).strip().upper().replace(' ', '')
    if not text:
        return []

    tasks = []
    for item in re.split(r'[,;]+', text):
        item = item.strip()
        if not item:
            continue

        match = re.match(r'^(?:SLOT)?([1-4])[:=@-]([A-Z0-9_]+)$', item)
        if match:
            tasks.append(task_from_slot_qr(match.group(1), match.group(2), default_lab))
            continue

        match = re.match(r'^([ABC]+)[:=@-]([1-4])$', item)
        if match:
            tasks.append(task_from_windows_lab(match.group(1), match.group(2)))
            continue

        match = re.match(r'^([1-4])([ABC]+)$', item)
        if match:
            tasks.append(task_from_slot_qr(match.group(1), match.group(2), default_lab))
            continue

        match = re.match(r'^([ABC]+)([1-4])$', item)
        if match:
            tasks.append(task_from_windows_lab(match.group(1), match.group(2)))
            continue

        windows = compact_windows(item)
        if windows:
            tasks.append(task_from_windows_lab(''.join(windows), default_lab))

    return dedupe_tasks(tasks)


def dedupe_tasks(tasks):
    out = []
    seen = set()
    for task in tasks:
        key = (task['target_lab'], ''.join(task['windows']))
        if key in seen:
            continue
        seen.add(key)
        out.append(task)
    return out


def parse_board_1_tasks(payload, default_lab='1'):
    text = to_text(payload).strip()
    if not text:
        return [task_from_windows_lab('A', default_lab)]

    fields = parse_status_fields(text)
    if 'tasks' in fields:
        tasks = parse_task_list(fields['tasks'], default_lab)
        if tasks:
            return tasks

    if 'slot' in fields and 'qr' in fields:
        return [task_from_slot_qr(fields['slot'], fields['qr'], default_lab)]

    if 'selected_slot' in fields and 'qr' in fields:
        return [task_from_slot_qr(fields['selected_slot'], fields['qr'], default_lab)]

    if 'target_lab' in fields and ('windows' in fields or 'qr' in fields):
        return [task_from_windows_lab(fields.get('windows', fields.get('qr', 'A')), fields['target_lab'])]

    tasks = parse_task_list(text, default_lab)
    if tasks:
        return tasks

    return [task_from_windows_lab('A', default_lab)]


def format_board_1_tasks(tasks):
    return ','.join('%s:%s' % (task['target_lab'], ''.join(task['windows'])) for task in tasks)


def parse_board_2(payload, default_wait=5, max_wait=30):
    text = to_text(payload).strip()
    upper = text.upper()
    fields = parse_status_fields(text)

    status = fields.get('status', '').strip().lower()
    wait_text = fields.get('wait', fields.get('wait_time', '')).strip()
    window_id = fields.get('window', fields.get('lab', '')).strip()

    free_words = [
        'FREE',
        'IDLE',
        'EMPTY',
        u'\u7a7a\u95f2',
        u'\u7a7a\u9592',
    ]
    busy_words = [
        'BUSY',
        'WAIT',
        u'\u5fd9',
        u'\u5fd9\u788c',
        u'\u7b49\u5f85',
    ]

    window_match = re.search(
        u'(?:WINDOW|WIN|LAB|\u7a97\u53e3|\u7a97|\u5316\u9a8c\u7a97\u53e3)\\s*([1-4])',
        upper + ' ' + text,
        re.IGNORECASE,
    )
    if not window_id and window_match:
        window_id = window_match.group(1)

    if status == 'free' or any(word in upper or word in text for word in free_words):
        spoken = u'\u5316\u9a8c\u533a\u7a7a\u95f2\uff0c\u8bf7\u5feb\u901f\u901a\u8fc7'
        if window_id:
            spoken = u'\u5316\u9a8c\u7a97\u53e3%s\u7a7a\u95f2\uff0c\u8bf7\u5feb\u901f\u901a\u8fc7' % window_id
        return {
            'status': 'free',
            'wait_time': 0,
            'window': window_id,
            'spoken': spoken,
        }

    wait_match = re.search(u'(?:WAIT|WAITS|WAITING|\u7b49\u5f85|\u7b49)\\D*([0-9]+)\\D*(?:S|SEC|SECOND|SECONDS|\u79d2)?', upper + ' ' + text)
    second_match = re.search(u'([0-9]+)\\D*(?:S|SEC|SECOND|SECONDS|\u79d2)', upper + ' ' + text)
    nums = re.findall(r'\d+', wait_text or text)
    if wait_text:
        wait_time = int(re.findall(r'\d+', wait_text)[0]) if re.findall(r'\d+', wait_text) else int(default_wait)
    elif wait_match:
        wait_time = int(wait_match.group(1))
    elif second_match:
        wait_time = int(second_match.group(1))
    elif len(nums) >= 2 and window_id and nums[0] == window_id:
        wait_time = int(nums[-1])
    elif nums:
        wait_time = int(nums[-1])
    else:
        wait_time = int(default_wait)
    wait_time = max(0, min(int(max_wait), wait_time))

    if status == 'busy' or nums or any(word in upper or word in text for word in busy_words):
        spoken = u'\u5316\u9a8c\u533a\u5fd9\u788c\uff0c\u9700\u7b49\u5f85%d\u79d2' % wait_time
        if window_id:
            spoken = u'\u5316\u9a8c\u7a97\u53e3%s\u5fd9\u788c\uff0c\u9700\u7b49\u5f85%d\u79d2' % (window_id, wait_time)
        return {
            'status': 'busy',
            'wait_time': wait_time,
            'window': window_id,
            'spoken': spoken,
        }

    return {
        'status': 'busy',
        'wait_time': int(default_wait),
        'window': window_id,
        'spoken': u'\u5316\u9a8c\u533a\u5fd9\u788c\uff0c\u9700\u7b49\u5f85%d\u79d2' % int(default_wait),
    }


class PatrolMission(object):
    def __init__(self):
        self.waypoints = load_waypoints(rospy.get_param('~waypoints'))
        self.send_goals = param_bool(rospy.get_param('~send_goals', False))
        self.board1 = rospy.get_param('~board1', 'A:1')
        self.board2 = rospy.get_param('~board2', 'FREE')
        self.timeout = float(rospy.get_param('~timeout', 120.0))

        self.use_live_board1 = param_bool(rospy.get_param('~use_live_board1', False))
        self.use_live_board2 = param_bool(rospy.get_param('~use_live_board2', False))
        self.board1_topic = rospy.get_param('~board1_topic', '/smart_pharmacy_patrol/board1_tasks')
        self.board2_topic = rospy.get_param('~board2_topic', '/smart_pharmacy_patrol/board2_status')
        self.board1_wait_timeout = float(rospy.get_param('~board1_wait_timeout', 12.0))
        self.board2_wait_timeout = float(rospy.get_param('~board2_wait_timeout', 12.0))
        self.live_max_age = float(rospy.get_param('~live_max_age', 6.0))
        self.default_lab = rospy.get_param('~default_lab', '1')
        self.execute_all_board1_tasks = param_bool(rospy.get_param('~execute_all_board1_tasks', True))
        self.max_board1_tasks = int(rospy.get_param('~max_board1_tasks', 2))
        self.board2_default_wait = int(rospy.get_param('~board2_default_wait', 5))
        self.board2_max_wait = int(rospy.get_param('~board2_max_wait', 30))
        self.board2_free_pass_sec = float(rospy.get_param('~board2_free_pass_sec', 0.5))
        self.board2_apply_only_matching_window = param_bool(
            rospy.get_param('~board2_apply_only_matching_window', True)
        )

        self.latest_board1 = ''
        self.latest_board1_time = 0.0
        self.latest_board2 = ''
        self.latest_board2_time = 0.0

        self.status_pub = rospy.Publisher('/smart_pharmacy_patrol/status', String, queue_size=10)
        self.voice_pub = rospy.Publisher('/smart_pharmacy_patrol/voice', String, queue_size=10)
        self.client = None

        rospy.Subscriber(self.board1_topic, String, self.on_board1, queue_size=10)
        rospy.Subscriber(self.board2_topic, String, self.on_board2, queue_size=10)

        if self.send_goals:
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo('Waiting for move_base...')
            if not self.client.wait_for_server(rospy.Duration(30.0)):
                raise RuntimeError('move_base action server is not available')

    def on_board1(self, msg):
        self.latest_board1 = to_text(msg.data).strip()
        self.latest_board1_time = time.time()

    def on_board2(self, msg):
        self.latest_board2 = to_text(msg.data).strip()
        self.latest_board2_time = time.time()

    def publish_status(self, text):
        text = to_text(text)
        self.status_pub.publish(String(data=ros_string(text)))
        log_text('[STATUS] ', text)

    def speak(self, text):
        text = to_text(text)
        self.voice_pub.publish(String(data=ros_string(text)))
        log_text('[VOICE] ', text)

    def go(self, waypoint_name, dwell=1.5):
        if waypoint_name not in self.waypoints:
            raise RuntimeError('Waypoint not found: ' + waypoint_name)

        point = self.waypoints[waypoint_name]
        self.publish_status('navigating:%s' % waypoint_name)
        rospy.loginfo(
            'NAV %s x=%.3f y=%.3f theta=%.3f',
            waypoint_name,
            float(point['x']),
            float(point['y']),
            float(point['theta']),
        )

        if not self.send_goals:
            rospy.sleep(dwell)
            return

        self.client.send_goal(make_goal(point))
        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        if not finished:
            self.client.cancel_goal()
            raise RuntimeError('Navigation timeout at waypoint: ' + waypoint_name)

        state = self.client.get_state()
        state_name = GOAL_STATUS_NAMES.get(state, str(state))
        result_text = self.client.get_goal_status_text()
        if state != GoalStatus.SUCCEEDED:
            self.client.cancel_goal()
            self.publish_status('navigation_failed:%s;state=%s' % (waypoint_name, state_name))
            raise RuntimeError(
                'Navigation failed at waypoint: %s; state=%s; text=%s'
                % (waypoint_name, state_name, result_text)
            )

        self.publish_status('navigation_succeeded:%s' % waypoint_name)
        rospy.sleep(dwell)

    def wait_for_live_payload(self, kind, timeout):
        start = time.time()
        while not rospy.is_shutdown() and time.time() - start < timeout:
            now = time.time()
            if kind == 'board1':
                if self.latest_board1 and now - self.latest_board1_time <= self.live_max_age:
                    return self.latest_board1
            else:
                if self.latest_board2 and now - self.latest_board2_time <= self.live_max_age:
                    return self.latest_board2
            rospy.sleep(0.05)
        return ''

    def get_board1_tasks(self):
        payload = ''
        if self.use_live_board1:
            payload = self.wait_for_live_payload('board1', self.board1_wait_timeout)
            if not payload:
                rospy.logwarn('No live board1 result, falling back to board1 param: %s', self.board1)
        if not payload:
            payload = self.board1

        tasks = parse_board_1_tasks(payload, self.default_lab)
        if not self.execute_all_board1_tasks:
            tasks = tasks[:1]
        if self.max_board1_tasks > 0:
            tasks = tasks[:self.max_board1_tasks]
        return tasks, payload

    def get_board2_status(self):
        payload = ''
        if self.use_live_board2:
            payload = self.wait_for_live_payload('board2', self.board2_wait_timeout)
            if not payload:
                rospy.logwarn('No live board2 result, falling back to board2 param: %s', self.board2)
        if not payload:
            payload = self.board2
        return parse_board_2(payload, self.board2_default_wait, self.board2_max_wait), payload

    def ordered_pickup_windows(self, tasks):
        ordered = []
        for task in tasks:
            for window in task['windows']:
                if window not in ordered:
                    ordered.append(window)
        return ordered

    def run_once(self):
        self.publish_status('mission_started;send_goals=%s' % self.send_goals)

        self.go('board_1')
        self.publish_status('recognizing:board_1')
        tasks, board1_payload = self.get_board1_tasks()
        task_text = format_board_1_tasks(tasks)
        first = tasks[0]
        self.publish_status(
            'vision_board_1:tasks=%s;selected_slot=%s;qr=%s;windows=%s;target_lab=%s'
            % (
                task_text,
                first['target_lab'],
                first['qr'],
                ','.join(first['windows']),
                first['target_lab'],
            )
        )
        rospy.loginfo('BOARD1 payload=%s parsed_tasks=%s', board1_payload, task_text)

        collected = 0
        for window in self.ordered_pickup_windows(tasks):
            self.go(WINDOW_WAYPOINTS[window])
            target_labs = [
                task['target_lab']
                for task in tasks
                if window in task['windows']
            ]
            collected += len(target_labs)
            self.speak('pickup window %s for lab %s' % (window, ','.join(target_labs)))
            self.publish_status(
                'collected:%s;labs=%s;count=%d'
                % (window, ','.join(target_labs), collected)
            )

        self.go('board_2')
        self.publish_status('recognizing:board_2')
        board2, board2_payload = self.get_board2_status()
        self.speak(board2['spoken'])
        self.publish_status(
            'vision_board_2:status=%s;wait=%d;window=%s;text=%s'
            % (board2['status'], board2['wait_time'], board2.get('window', ''), board2_payload)
        )

        target_labs = set(task['target_lab'] for task in tasks)
        board2_window = to_text(board2.get('window', '')).strip()
        board2_applies = True
        if self.board2_apply_only_matching_window and board2_window:
            board2_applies = board2_window in target_labs
            if not board2_applies:
                self.publish_status(
                    'vision_board_2:ignored_window=%s;target_labs=%s'
                    % (board2_window, ','.join(sorted(target_labs)))
                )

        if board2_applies and board2['status'] == 'busy' and board2['wait_time'] > 0:
            rospy.sleep(board2['wait_time'])
        elif board2_applies and board2['status'] == 'free':
            rospy.sleep(min(3.0, max(0.0, self.board2_free_pass_sec)))

        for task in tasks:
            lab_id = task['target_lab']
            self.go('lab_window_' + lab_id)
            self.speak('arrived lab window %s, source %s' % (lab_id, ''.join(task['windows'])))
            self.publish_status(
                'dropped:lab=%s;windows=%s;qr=%s;count=%d'
                % (lab_id, ','.join(task['windows']), task['qr'], collected)
            )

        self.go('start', dwell=0.5)
        self.publish_status('cycle_complete')


def main():
    rospy.init_node('smart_pharmacy_patrol')
    mission = PatrolMission()
    mission.run_once()


if __name__ == '__main__':
    main()
