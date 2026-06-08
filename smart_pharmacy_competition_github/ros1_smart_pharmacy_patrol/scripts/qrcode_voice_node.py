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
    from pyzbar.pyzbar import decode
except ImportError:
    decode = None

try:
    unicode
except NameError:
    unicode = str

PY2 = sys.version_info[0] == 2


def to_text(value):
    if isinstance(value, unicode):
        return value
    try:
        return unicode(value, 'utf-8', errors='replace')
    except TypeError:
        return unicode(value)


def ros_string(text):
    text = to_text(text)
    if PY2:
        return text.encode('utf-8')
    return text


def param_bool(value):
    if isinstance(value, bool):
        return value
    return to_text(value).strip().lower() in ('1', 'true', 'yes', 'on')


def audio_key(text):
    key = to_text(text).strip().upper()
    key = re.sub(r'[^A-Z0-9]+', '_', key).strip('_')
    return key or 'UNKNOWN'


def audio_key_candidates(text):
    key = audio_key(text)
    candidates = [key]
    parts = [part for part in key.split('_') if part]
    if parts:
        candidates.append(parts[0])
    out = []
    for item in candidates:
        if item not in out:
            out.append(item)
    return out


def code_matches_allowlist(text, valid_codes):
    if not valid_codes:
        return True
    normalized = to_text(text).strip().upper()
    if normalized in valid_codes:
        return True

    key = audio_key(normalized)
    if key in valid_codes:
        return True

    # Accept compound task codes such as AB:1 when AB and/or 1 are allowed.
    parts = [part for part in key.split('_') if part]
    for part in parts:
        if part in valid_codes:
            return True

    windows = ''.join(char for char in normalized if char in 'ABC')
    digits = ''.join(char for char in normalized if char in '1234')
    if windows and windows in valid_codes:
        return True
    if digits and digits in valid_codes:
        return True
    return False


def normalize_qr_text(text):
    return to_text(text).strip().upper().replace(' ', '')


def compact_windows(text):
    windows = []
    for char in normalize_qr_text(text):
        if char in 'ABC' and char not in windows:
            windows.append(char)
    return ''.join(windows)


class QRCodeVoiceNode(object):
    def __init__(self):
        if decode is None:
            raise RuntimeError('pyzbar is not available; cannot decode QR codes')

        self.image_topics = [
            item.strip()
            for item in rospy.get_param(
                '~image_topics',
                '/camera/rgb/image_rect_color,/camera/rgb/image_raw',
            ).split(',')
            if item.strip()
        ]
        self.result_topic = rospy.get_param('~result_topic', '/vision_result')
        self.cv1_topic = rospy.get_param('~cv1_topic', '/judge/cv1')
        self.status_topic = rospy.get_param('~status_topic', '/smart_pharmacy_patrol/status')
        self.board1_tasks_topic = rospy.get_param(
            '~board1_tasks_topic',
            '/smart_pharmacy_patrol/board1_tasks',
        )
        self.audio_dir = rospy.get_param(
            '~audio_dir',
            os.path.join(os.path.dirname(os.path.dirname(__file__)), 'sounds', 'qr_voice'),
        )
        self.audio_prefix = rospy.get_param('~audio_prefix', 'qr_')
        self.generic_audio = rospy.get_param('~generic_audio', 'qr_detected.wav')
        self.audio_player = rospy.get_param('~audio_player', 'aplay')
        self.cooldown_sec = float(rospy.get_param('~cooldown_sec', 5.0))
        self.vote_window_sec = float(rospy.get_param('~vote_window_sec', 2.0))
        self.vote_frames = int(rospy.get_param('~vote_frames', 2))
        self.save_images = param_bool(rospy.get_param('~save_images', True))
        self.latest_save_period_sec = float(rospy.get_param('~latest_save_period_sec', 1.0))
        self.output_dir = rospy.get_param('~output_dir', '/home/EPRobot/vision_test')
        self.slot_mode = param_bool(rospy.get_param('~slot_mode', True))
        self.slot_x_split = float(rospy.get_param('~slot_x_split', 0.5))
        self.slot_y_split = float(rospy.get_param('~slot_y_split', 0.5))
        self.max_tasks = int(rospy.get_param('~max_tasks', 4))
        slot_order_text = to_text(rospy.get_param('~slot_order', '1,2,3,4')).strip()
        self.slot_order = [
            item.strip()
            for item in re.split(r'[,;| ]+', slot_order_text)
            if item.strip()
        ]
        if len(self.slot_order) != 4 or any(item not in ('1', '2', '3', '4') for item in self.slot_order):
            self.slot_order = ['1', '2', '3', '4']

        valid_codes = to_text(rospy.get_param('~valid_codes', '')).strip()
        self.valid_codes = set()
        if valid_codes:
            self.valid_codes = set(
                item.strip().upper()
                for item in re.split(r'[,;| ]+', valid_codes)
                if item.strip()
            )

        self.bridge = CvBridge()
        self.recent = deque()
        self.lock = threading.RLock()
        self.last_result = ''
        self.last_trigger_time = 0.0
        self.last_latest_save_time = 0.0
        self.last_play_process = None

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10)
        self.cv1_pub = rospy.Publisher(self.cv1_topic, String, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.board1_tasks_pub = rospy.Publisher(self.board1_tasks_topic, String, queue_size=10)

        if self.save_images and not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        for topic in self.image_topics:
            rospy.Subscriber(topic, Image, self.on_image, callback_args=topic, queue_size=1)

        rospy.loginfo(
            '[qrcode_voice] listening topics=%s audio_dir=%s slot_mode=%s',
            ','.join(self.image_topics),
            self.audio_dir,
            self.slot_mode,
        )

    def slot_from_center(self, x, y, width, height):
        if not self.slot_mode or width <= 0 or height <= 0:
            return ''
        col = 0 if float(x) < width * self.slot_x_split else 1
        row = 0 if float(y) < height * self.slot_y_split else 1
        index = row * 2 + col
        return self.slot_order[index]

    def code_center(self, code):
        rect = getattr(code, 'rect', None)
        if rect is not None:
            try:
                return (
                    float(rect.left) + float(rect.width) / 2.0,
                    float(rect.top) + float(rect.height) / 2.0,
                )
            except Exception:
                pass

        points = getattr(code, 'polygon', None) or []
        if points:
            try:
                xs = [float(point.x) for point in points]
                ys = [float(point.y) for point in points]
                return sum(xs) / len(xs), sum(ys) / len(ys)
            except Exception:
                pass
        return 0.0, 0.0

    def decode_frame(self, frame):
        variants = [frame]
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            variants.append(gray)
            variants.append(cv2.equalizeHist(gray))
            blur = cv2.GaussianBlur(gray, (0, 0), 1.0)
            variants.append(cv2.addWeighted(gray, 1.5, blur, -0.5, 0))
        except Exception:
            pass

        height, width = frame.shape[:2]
        by_key = {}
        for image in variants:
            for code in decode(image):
                text = normalize_qr_text(code.data)
                if not text:
                    continue
                if not code_matches_allowlist(text, self.valid_codes):
                    continue
                center_x, center_y = self.code_center(code)
                slot = self.slot_from_center(center_x, center_y, width, height)
                key = '%s:%s' % (slot, text) if slot else text
                if key not in by_key:
                    by_key[key] = {
                        'slot': slot,
                        'text': text,
                        'center_x': center_x,
                        'center_y': center_y,
                    }

        records = list(by_key.values())
        records.sort(key=lambda item: (int(item['slot']) if item['slot'] else 99, item['text']))
        if self.max_tasks > 0:
            records = records[:self.max_tasks]
        return records

    def format_records(self, records):
        parts = []
        for record in records:
            if record.get('slot'):
                parts.append('%s:%s' % (record['slot'], record['text']))
            else:
                parts.append(record['text'])
        return ','.join(parts)

    def on_image(self, msg, topic):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(2.0, '[qrcode_voice] cv_bridge failed on %s: %s', topic, exc)
            return

        now = time.time()
        if self.save_images and now - self.last_latest_save_time >= self.latest_save_period_sec:
            self.last_latest_save_time = now
            cv2.imwrite(os.path.join(self.output_dir, 'qr_voice_latest.jpg'), frame)

        records = self.decode_frame(frame)
        if not records:
            return
        aggregate = self.format_records(records)

        with self.lock:
            self.recent.append((now, aggregate, records))
            while self.recent and now - self.recent[0][0] > self.vote_window_sec:
                self.recent.popleft()

            counts = Counter(text for _, text, _ in self.recent)
            result, count = counts.most_common(1)[0]
            if count < self.vote_frames:
                return

            if result == self.last_result and now - self.last_trigger_time < self.cooldown_sec:
                return
            self.last_result = result
            self.last_trigger_time = now
            stable_records = []
            for _, text, items in reversed(self.recent):
                if text == result:
                    stable_records = items
                    break

        if self.save_images:
            annotated = self.annotate_frame(frame, stable_records)
            cv2.imwrite(os.path.join(self.output_dir, 'qr_voice_detected.jpg'), annotated)
        self.handle_result(result, stable_records)

    def annotate_frame(self, frame, records):
        image = frame.copy()
        height, width = image.shape[:2]
        if self.slot_mode:
            cv2.line(image, (int(width * self.slot_x_split), 0), (int(width * self.slot_x_split), height), (0, 255, 0), 1)
            cv2.line(image, (0, int(height * self.slot_y_split)), (width, int(height * self.slot_y_split)), (0, 255, 0), 1)
        for record in records:
            label = '%s:%s' % (record.get('slot') or '?', record.get('text') or '')
            cv2.putText(
                image,
                label,
                (int(record.get('center_x', 0)), int(record.get('center_y', 0))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
        return image

    def handle_result(self, result, records=None):
        result = to_text(result).strip()
        records = records or []
        rospy.loginfo('[qrcode_voice] QR_RESULT=%s', result.encode('utf-8') if PY2 else result)

        self.result_pub.publish(String(data=ros_string('QR:' + result)))
        self.cv1_pub.publish(String(data=ros_string(result)))
        self.board1_tasks_pub.publish(String(data=ros_string(result)))

        if records:
            selected = records[0]
            slot = selected.get('slot') or '1'
            qr = selected.get('text') or result
            windows = compact_windows(qr) or qr
            status = (
                'vision_board_1:tasks=%s;selected_slot=%s;qr=%s;windows=%s;target_lab=%s'
                % (result, slot, qr, windows, slot)
            )
            audio_result = '%s:%s' % (windows, slot)
        else:
            status = 'vision_board_1:qr=' + result
            audio_result = result
        self.status_pub.publish(String(data=ros_string(status)))
        self.play_audio(audio_result)

    def choose_audio_file(self, result):
        candidates = []
        for key in audio_key_candidates(result):
            candidates.append(os.path.join(self.audio_dir, self.audio_prefix + key + '.wav'))
        candidates.append(os.path.join(self.audio_dir, self.generic_audio))
        for path in candidates:
            if path and os.path.isfile(path):
                return path
        return ''

    def play_audio(self, result):
        path = self.choose_audio_file(result)
        if not path:
            rospy.logwarn('[qrcode_voice] no audio file found for QR=%s', result)
            return

        try:
            if self.last_play_process is not None and self.last_play_process.poll() is None:
                self.last_play_process.terminate()
        except Exception:
            pass

        try:
            self.last_play_process = subprocess.Popen(
                [self.audio_player, '-q', path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            rospy.loginfo('[qrcode_voice] playing %s', path)
        except Exception as exc:
            rospy.logwarn('[qrcode_voice] audio play failed: %s', exc)


def main():
    rospy.init_node('qrcode_voice')
    QRCodeVoiceNode()
    rospy.spin()


if __name__ == '__main__':
    main()
