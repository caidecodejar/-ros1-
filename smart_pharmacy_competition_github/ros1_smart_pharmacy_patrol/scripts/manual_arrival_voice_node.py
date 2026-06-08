#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import re
import subprocess
import sys
import time

import rospy
from std_msgs.msg import String

from patrol_mission import format_board_1_tasks
from patrol_mission import parse_board_1_tasks
from patrol_mission import ros_string
from patrol_mission import to_text

try:
    unicode
except NameError:
    unicode = str


SAMPLE_BY_LAB = {
    '1': u'\u9759\u8109\u8840\u6837\u672c',
    '2': u'\u5507\u6db2\u6837\u672c',
    '3': u'\u7ec4\u7ec7\u6837\u672c',
    '4': u'\u8840\u6d46\u6837\u672c',
}

LAB_NAME_BY_ID = {
    '1': u'\u8840\u5e38\u89c4\u7a97\u53e3',
    '2': u'\u4f53\u6db2\u7a97\u53e3',
    '3': u'\u514d\u75ab\u68c0\u6d4b\u7a97\u53e3',
    '4': u'\u6fc0\u7d20\u68c0\u9a8c\u7a97\u53e3',
}


def param_bool(value):
    if isinstance(value, bool):
        return value
    return to_text(value).strip().lower() in ('1', 'true', 'yes', 'on')


def clean_window(value):
    text = to_text(value).strip().upper()
    compact = re.sub(r'[^A-Z0-9_]+', '', text)
    compact = compact.replace('__', '_').strip('_')
    if compact in ('A', 'B', 'C'):
        return compact
    match = re.match(r'^(?:WINDOW|WIN)_?([ABC])$', compact)
    if match:
        return match.group(1)
    match = re.match(r'^([ABC])_?(?:WINDOW|WIN)$', compact)
    if match:
        return match.group(1)
    return ''


def clean_lab(value):
    text = to_text(value).strip().lower()
    text = text.replace('lab_window_', '').replace('lab', '').replace('window_', '')
    match = re.search(r'[1-4]', text)
    return match.group(0) if match else ''


def join_cn(items):
    items = [to_text(item) for item in items if to_text(item).strip()]
    if not items:
        return ''
    return u'\u3001'.join(items)


class ManualArrivalVoiceNode(object):
    def __init__(self):
        self.default_board1 = to_text(rospy.get_param('~default_board1', '')).strip()
        self.default_lab = to_text(rospy.get_param('~default_lab', '1')).strip() or '1'
        self.board1_topic = rospy.get_param('~board1_topic', '/smart_pharmacy_patrol/board1_tasks')
        self.cv1_topic = rospy.get_param('~cv1_topic', '/judge/cv1')
        self.arrival_topic = rospy.get_param('~arrival_topic', '/smart_pharmacy_patrol/manual_arrival')
        self.status_topic = rospy.get_param('~status_topic', '/smart_pharmacy_patrol/status')
        self.voice_topic = rospy.get_param('~voice_topic', '/smart_pharmacy_patrol/voice')
        self.task_topic = rospy.get_param('~task_topic', '/judge/task')
        self.audio_command = to_text(rospy.get_param('~audio_command', '')).strip()
        self.enable_audio = param_bool(rospy.get_param('~enable_audio', True))
        self.repeat_pickup = param_bool(rospy.get_param('~repeat_pickup', False))

        self.latest_board1 = self.default_board1
        self.latest_board1_time = 0.0
        self.tasks = self.parse_tasks(self.latest_board1)
        self.collected_pairs = set()
        self.collected_by_lab = dict((lab, set()) for lab in SAMPLE_BY_LAB)
        self.last_instruction_payload = ''
        self.last_play_process = None

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.voice_pub = rospy.Publisher(self.voice_topic, String, queue_size=10)
        self.task_pub = rospy.Publisher(self.task_topic, String, queue_size=10)

        rospy.Subscriber(self.board1_topic, String, self.on_board1, queue_size=10)
        rospy.Subscriber(self.cv1_topic, String, self.on_board1, queue_size=10)
        rospy.Subscriber(self.arrival_topic, String, self.on_arrival, queue_size=10)

        rospy.loginfo(
            '[manual_arrival_voice] ready arrival_topic=%s board1_topic=%s cv1_topic=%s',
            self.arrival_topic,
            self.board1_topic,
            self.cv1_topic,
        )
        if self.latest_board1:
            self.publish_board1_status()

    def parse_tasks(self, payload):
        payload = to_text(payload).strip()
        if not payload:
            return []
        try:
            return parse_board_1_tasks(payload, self.default_lab)
        except Exception as exc:
            rospy.logwarn('[manual_arrival_voice] board1 parse failed: %s', exc)
            return []

    def publish_status(self, text):
        text = to_text(text)
        self.status_pub.publish(String(data=ros_string(text)))
        rospy.loginfo('[manual_arrival_voice] status %s', text)

    def publish_task(self, task):
        self.task_pub.publish(String(data=ros_string(task)))

    def publish_board1_status(self):
        if not self.tasks:
            return
        task_text = format_board_1_tasks(self.tasks)
        first = self.tasks[0]
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

    def speak_board1_instruction(self, payload):
        payload = to_text(payload).strip()
        if not self.tasks or payload == self.last_instruction_payload:
            return
        self.last_instruction_payload = payload

        phrases = []
        for task in self.tasks:
            lab = task.get('target_lab', self.default_lab)
            sample = SAMPLE_BY_LAB.get(lab, u'\u672a\u77e5\u6837\u672c')
            windows = join_cn(task.get('windows', []))
            if not windows:
                continue
            phrases.append(u'\u8bf7\u524d\u5f80%s\u7a97\u53e3\u53d6%s' % (windows, sample))
        if not phrases:
            return
        self.speak(u'\u8bc6\u522b\u677f\u4e00\u8bc6\u522b\u6210\u529f\uff0c' + u'\uff1b'.join(phrases))

    def speak(self, text):
        text = to_text(text)
        self.voice_pub.publish(String(data=ros_string(text)))
        rospy.loginfo('[manual_arrival_voice] voice %s', text)
        if not self.enable_audio:
            return

        commands = []
        if self.audio_command:
            commands.append(self.audio_command.split() + [text])
        commands.extend([
            ['espeak-ng', '-v', 'zh', '-s', '140', text],
            ['espeak', '-v', 'zh', '-s', '140', text],
            ['spd-say', text],
        ])

        for command in commands:
            try:
                if self.last_play_process is not None and self.last_play_process.poll() is None:
                    self.last_play_process.terminate()
                self.last_play_process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                return
            except Exception:
                continue
        rospy.logwarn_throttle(10.0, '[manual_arrival_voice] no text-to-speech command available')

    def on_board1(self, msg):
        payload = to_text(msg.data).strip()
        if not payload:
            return
        self.latest_board1 = payload
        self.latest_board1_time = time.time()
        self.tasks = self.parse_tasks(payload)
        rospy.loginfo(
            '[manual_arrival_voice] board1=%s parsed=%s',
            payload,
            format_board_1_tasks(self.tasks) if self.tasks else '',
        )
        self.publish_board1_status()
        self.speak_board1_instruction(payload)

    def on_arrival(self, msg):
        text = to_text(msg.data).strip()
        window = clean_window(text)
        lab = clean_lab(text)
        if window:
            self.handle_pickup(window)
            return
        if lab:
            self.handle_lab(lab)
            return
        self.speak(u'\u672a\u8bc6\u522b\u5230\u5230\u70b9\u7c7b\u578b')
        self.publish_status('manual_arrival:unknown;text=%s' % text)

    def tasks_for_window(self, window):
        return [task for task in self.tasks if window in task.get('windows', [])]

    def tasks_for_lab(self, lab):
        return [task for task in self.tasks if task.get('target_lab') == lab]

    def handle_pickup(self, window):
        self.publish_task(window)
        tasks = self.tasks_for_window(window)
        if not tasks:
            self.speak(u'\u5230\u8fbe%s\u7a97\u53e3\uff0c\u672a\u5339\u914d\u5230\u8bc6\u522b\u677f\u4e00\u4efb\u52a1' % window)
            self.publish_status('arrived_window:%s;matched=false' % window)
            return

        new_labs = []
        all_labs = []
        for task in tasks:
            lab = task['target_lab']
            all_labs.append(lab)
            key = (window, lab)
            if self.repeat_pickup or key not in self.collected_pairs:
                self.collected_pairs.add(key)
                self.collected_by_lab.setdefault(lab, set()).add(window)
                new_labs.append(lab)

        labs_for_voice = new_labs or all_labs
        sample_names = [SAMPLE_BY_LAB.get(lab, u'\u672a\u77e5\u6837\u672c') for lab in labs_for_voice]
        sample_text = join_cn(sample_names)
        total_count = len(self.collected_pairs)
        spoken = u'\u5230\u8fbe%s\u7a97\u53e3\uff0c\u53d6\u5230%s\u7a97\u53e3\u4e2d\u7684%s' % (
            window,
            window,
            sample_text,
        )
        if not new_labs:
            spoken = u'\u5230\u8fbe%s\u7a97\u53e3\uff0c%s\u5df2\u8bb0\u5f55' % (window, sample_text)
        self.speak(spoken)
        self.publish_status(
            'collected:%s;labs=%s;samples=%s;count=%d'
            % (window, ','.join(labs_for_voice), sample_text, total_count)
        )

    def handle_lab(self, lab):
        self.publish_task(lab)
        lab_name = LAB_NAME_BY_ID.get(lab, u'\u672a\u77e5\u5316\u9a8c\u7a97\u53e3')
        sample_name = SAMPLE_BY_LAB.get(lab, u'\u672a\u77e5\u6837\u672c')

        collected_windows = sorted(self.collected_by_lab.get(lab, set()))
        if collected_windows:
            count = len(collected_windows)
            windows = collected_windows
        else:
            tasks = self.tasks_for_lab(lab)
            windows = []
            for task in tasks:
                for window in task.get('windows', []):
                    if window not in windows:
                        windows.append(window)
            count = len(windows)

        windows_text = ','.join(windows)
        spoken = u'\u5230\u8fbe%s\uff0c\u9001\u8fbe%s\uff0c\u6837\u672c\u6570\u4e3a%d' % (
            lab_name,
            sample_name,
            count,
        )
        self.speak(spoken)
        self.publish_status(
            'dropped:lab=%s;windows=%s;sample=%s;count=%d'
            % (lab, windows_text, sample_name, count)
        )


def main():
    rospy.init_node('manual_arrival_voice')
    ManualArrivalVoiceNode()
    rospy.spin()


if __name__ == '__main__':
    main()
