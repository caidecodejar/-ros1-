#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(description='Minimal TCP server for judge bridge tests.')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=8888, type=int)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(5)
    print('fake judge listening on %s:%d' % (args.host, args.port))
    try:
        while True:
            conn, addr = server.accept()
            print('client', addr)
            data = b''
            try:
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                    while b'\n' in data:
                        line, data = data.split(b'\n', 1)
                        text = line.decode('utf-8', 'replace').strip()
                        if not text:
                            continue
                        if text.startswith('CRASH'):
                            print('CRASH')
                            continue
                        try:
                            print(json.dumps(json.loads(text), ensure_ascii=False, sort_keys=True))
                        except Exception:
                            print('RAW', text)
            finally:
                conn.close()

            if args.once:
                break
    finally:
        server.close()


if __name__ == '__main__':
    main()
