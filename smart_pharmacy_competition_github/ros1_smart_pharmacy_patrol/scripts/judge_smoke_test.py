#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import socket
import time


def parse_odom(value):
    parts = [item.strip() for item in value.split(',')]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError('odom must be x,y')
    return [float(parts[0]), float(parts[1])]


def main():
    parser = argparse.ArgumentParser(description='Send newline-delimited judge JSON test packets.')
    parser.add_argument('--host', default='192.168.12.248')
    parser.add_argument('--port', default=8888, type=int)
    parser.add_argument('--source-ip', default='', help='Optional local source IP, normally 192.168.12.1 on the robot.')
    parser.add_argument('--count', default=3, type=int)
    parser.add_argument('--interval', default=0.2, type=float)
    parser.add_argument('--task', default='A')
    parser.add_argument('--speed', default=0.08, type=float)
    parser.add_argument('--odom', default=[1.20, 2.30], type=parse_odom)
    parser.add_argument('--odom-x', default=None, type=float)
    parser.add_argument('--odom-y', default=None, type=float)
    parser.add_argument('--cv1', default='AB:1')
    parser.add_argument('--cv2', default='FREE')
    parser.add_argument('--crash', action='store_true')
    args = parser.parse_args()
    if args.odom_x is not None or args.odom_y is not None:
        if args.odom_x is None or args.odom_y is None:
            parser.error('--odom-x and --odom-y must be used together')
        args.odom = [args.odom_x, args.odom_y]

    source = (args.source_ip, 0) if args.source_ip else None
    sock = socket.create_connection((args.host, args.port), timeout=5, source_address=source)
    try:
        if args.crash:
            sock.sendall(b'CRASH\n')
            print('sent CRASH')

        for idx in range(args.count):
            payload = {
                'task': args.task,
                'speed': round(args.speed, 3),
                'odom': [round(args.odom[0], 3), round(args.odom[1], 3)],
                'CV1': args.cv1,
                'CV2': args.cv2,
            }
            line = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
            if not isinstance(line, bytes):
                line = line.encode('utf-8')
            sock.sendall(line + b'\n')
            print('sent', idx + 1, line.decode('utf-8'))
            time.sleep(args.interval)
    finally:
        sock.close()


if __name__ == '__main__':
    main()
