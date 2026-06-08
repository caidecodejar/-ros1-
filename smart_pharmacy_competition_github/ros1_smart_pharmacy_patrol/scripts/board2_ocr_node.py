#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import re
import subprocess
import sys
import threading
import time
from collections import Counter, deque

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import pytesseract
except ImportError:
    pytesseract = None

try:
    unicode
except NameError:
    unicode = str

PY2 = sys.version_info[0] == 2


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


def parse_board2_text(text, default_wait=5, max_wait=30):
    text = to_text(text).strip()
    upper = text.upper()
    compact = re.sub(r'\s+', '', text)
    search_text = upper + ' ' + compact

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
        search_text,
        re.IGNORECASE,
    )
    window_id = window_match.group(1) if window_match else ''

    if any(word in upper or word in compact for word in free_words):
        spoken = u'\u8bc6\u522b\u5230\u5316\u9a8c\u7a97\u53e3\u7a7a\u95f2'
        if window_id:
            spoken = u'\u8bc6\u522b\u5230\u5316\u9a8c\u7a97\u53e3%s\u7a7a\u95f2' % window_id
        return {
            'status': 'free',
            'wait_time': 0,
            'window': window_id,
            'cv2': 'FREE',
            'spoken': spoken,
        }

    nums = re.findall(r'\d+', compact)
    wait_match = re.search(u'(?:WAIT|WAITS|WAITING|\u7b49\u5f85|\u7b49)\\D*([0-9]+)\\D*(?:S|SEC|SECOND|SECONDS|\u79d2)?', search_text)
    second_match = re.search(u'([0-9]+)\\D*(?:S|SEC|SECOND|SECONDS|\u79d2)', search_text)
    if wait_match:
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
    if any(word in upper or word in compact for word in busy_words) or nums:
        spoken = u'\u8bc6\u522b\u5230\u5316\u9a8c\u7a97\u53e3\u5fd9\u788c\uff0c\u7b49\u5f85%d\u79d2' % wait_time
        if window_id:
            spoken = u'\u8bc6\u522b\u5230\u5316\u9a8c\u7a97\u53e3%s\u5fd9\u788c\uff0c\u7b49\u5f85%d\u79d2' % (window_id, wait_time)
        return {
            'status': 'busy',
            'wait_time': wait_time,
            'window': window_id,
            'cv2': 'BUSY:%d' % wait_time,
            'spoken': spoken,
        }

    return None


class Board2OcrNode(object):
    def __init__(self):
        if pytesseract is None:
            raise RuntimeError('pytesseract is not available; cannot OCR board 2')

        self.image_topics = [
            item.strip()
            for item in rospy.get_param(
                '~image_topics',
                '/camera/rgb/image_rect_color,/camera/rgb/image_raw',
            ).split(',')
            if item.strip()
        ]
        self.result_topic = rospy.get_param('~result_topic', '/vision_result')
        self.cv2_topic = rospy.get_param('~cv2_topic', '/judge/cv2')
        self.board2_status_topic = rospy.get_param(
            '~board2_status_topic',
            '/smart_pharmacy_patrol/board2_status',
        )
        self.status_topic = rospy.get_param('~status_topic', '/smart_pharmacy_patrol/status')
        self.voice_topic = rospy.get_param('~voice_topic', '/smart_pharmacy_patrol/voice')

        self.cooldown_sec = float(rospy.get_param('~cooldown_sec', 4.0))
        self.vote_window_sec = float(rospy.get_param('~vote_window_sec', 2.5))
        self.vote_frames = int(rospy.get_param('~vote_frames', 2))
        self.default_wait = int(rospy.get_param('~default_wait', 5))
        self.max_wait = int(rospy.get_param('~max_wait', 30))
        self.lang = rospy.get_param('~lang', 'chi_sim+eng')
        self.tesseract_config = rospy.get_param('~tesseract_config', '--psm 6')
        self.roi_x1 = float(rospy.get_param('~roi_x1', 0.10))
        self.roi_y1 = float(rospy.get_param('~roi_y1', 0.10))
        self.roi_x2 = float(rospy.get_param('~roi_x2', 0.90))
        self.roi_y2 = float(rospy.get_param('~roi_y2', 0.90))
        self.save_images = param_bool(rospy.get_param('~save_images', True))
        self.latest_save_period_sec = float(rospy.get_param('~latest_save_period_sec', 1.0))
        self.output_dir = rospy.get_param('~output_dir', '/home/EPRobot/vision_test')
        self.enable_audio = param_bool(rospy.get_param('~enable_audio', True))
        self.audio_command = rospy.get_param('~audio_command', '')

        self.bridge = CvBridge()
        self.recent = deque()
        self.lock = threading.RLock()
        self.last_result = ''
        self.last_trigger_time = 0.0
        self.last_latest_save_time = 0.0
        self.last_play_process = None

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10)
        self.cv2_pub = rospy.Publisher(self.cv2_topic, String, queue_size=10)
        self.board2_status_pub = rospy.Publisher(self.board2_status_topic, String, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.voice_pub = rospy.Publisher(self.voice_topic, String, queue_size=10)

        if self.save_images and not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        for topic in self.image_topics:
            rospy.Subscriber(topic, Image, self.on_image, callback_args=topic, queue_size=1)

        rospy.loginfo(
            '[board2_ocr] listening topics=%s lang=%s roi=(%.2f,%.2f,%.2f,%.2f)',
            ','.join(self.image_topics),
            self.lang,
            self.roi_x1,
            self.roi_y1,
            self.roi_x2,
            self.roi_y2,
        )

    def crop_roi(self, frame):
        height, width = frame.shape[:2]
        x1 = max(0, min(width - 1, int(width * self.roi_x1)))
        y1 = max(0, min(height - 1, int(height * self.roi_y1)))
        x2 = max(x1 + 1, min(width, int(width * self.roi_x2)))
        y2 = max(y1 + 1, min(height, int(height * self.roi_y2)))
        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)

    def preprocess(self, roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary

    def ocr_frame(self, frame):
        roi, rect = self.crop_roi(frame)
        image = self.preprocess(roi)
        try:
            text = pytesseract.image_to_string(image, lang=self.lang, config=self.tesseract_config)
        except Exception as exc:
            rospy.logwarn_throttle(3.0, '[board2_ocr] tesseract failed: %s', exc)
            return '', None, rect, image

        parsed = parse_board2_text(text, self.default_wait, self.max_wait)
        return text, parsed, rect, image

    def on_image(self, msg, topic):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(2.0, '[board2_ocr] cv_bridge failed on %s: %s', topic, exc)
            return

        now = time.time()
        if self.save_images and now - self.last_latest_save_time >= self.latest_save_period_sec:
            self.last_latest_save_time = now
            cv2.imwrite(os.path.join(self.output_dir, 'board2_latest.jpg'), frame)

        raw_text, parsed, rect, processed = self.ocr_frame(frame)
        if parsed is None:
            return

        key = '%s:%d' % (parsed['status'], parsed['wait_time'])
        with self.lock:
            self.recent.append((now, key, raw_text, parsed, rect, processed))
            while self.recent and now - self.recent[0][0] > self.vote_window_sec:
                self.recent.popleft()

            counts = Counter(item[1] for item in self.recent)
            result, count = counts.most_common(1)[0]
            if count < self.vote_frames:
                return
            if result == self.last_result and now - self.last_trigger_time < self.cooldown_sec:
                return
            self.last_result = result
            self.last_trigger_time = now

            stable = None
            for item in reversed(self.recent):
                if item[1] == result:
                    stable = item
                    break

        if stable is None:
            return
        _, _, raw_text, parsed, rect, processed = stable
        if self.save_images:
            annotated = frame.copy()
            x1, y1, x2, y2 = rect
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = '%s:%d' % (parsed['status'], parsed['wait_time'])
            cv2.putText(annotated, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imwrite(os.path.join(self.output_dir, 'board2_detected.jpg'), annotated)
            cv2.imwrite(os.path.join(self.output_dir, 'board2_processed.jpg'), processed)

        self.handle_result(parsed, raw_text)

    def handle_result(self, parsed, raw_text):
        safe_text = re.sub(r'\s+', ' ', to_text(raw_text)).strip()
        status_line = 'status=%s;wait=%d;text=%s' % (
            parsed['status'],
            parsed['wait_time'],
            safe_text,
        )
        if parsed.get('window'):
            status_line = 'status=%s;wait=%d;window=%s;text=%s' % (
                parsed['status'],
                parsed['wait_time'],
                parsed['window'],
                safe_text,
            )
        mission_line = 'vision_board_2:' + status_line
        result_line = 'STATUS:%s:%d' % (parsed['status'].upper(), parsed['wait_time'])

        rospy.loginfo('[board2_ocr] %s raw=%s', status_line, safe_text)
        self.result_pub.publish(String(data=ros_string(result_line)))
        self.cv2_pub.publish(String(data=ros_string(parsed['cv2'])))
        self.board2_status_pub.publish(String(data=ros_string(status_line)))
        self.status_pub.publish(String(data=ros_string(mission_line)))
        self.voice_pub.publish(String(data=ros_string(parsed['spoken'])))
        self.play_audio(parsed['spoken'])

    def play_audio(self, text):
        if not self.enable_audio:
            return
        commands = []
        if self.audio_command:
            commands.append(self.audio_command.split() + [to_text(text)])
        commands.extend([
            ['espeak-ng', '-v', 'zh', '-s', '140', to_text(text)],
            ['espeak', '-v', 'zh', '-s', '140', to_text(text)],
            ['spd-say', to_text(text)],
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
        rospy.logwarn_throttle(10.0, '[board2_ocr] no text-to-speech command available')


def main():
    rospy.init_node('board2_ocr')
    Board2OcrNode()
    rospy.spin()


if __name__ == '__main__':
    main()
