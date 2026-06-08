#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import getpass
import os
from pathlib import Path
import shlex
import socket
import sys

import paramiko


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKAGE = ROOT / 'judge_system_match_ready_20260605_manual.tar.gz'


REMOTE_SCRIPT = r'''#!/usr/bin/env bash
set -e
source ~/eprobot_env.sh 2>/dev/null || true
source /opt/ros/*/setup.bash 2>/dev/null || true

mkdir -p ~/robot_ws/src
REMOTE_PACKAGE="${REMOTE_PACKAGE:-/home/EPRobot/judge_system_match_ready_20260605_manual.tar.gz}"
tar -xzf "$REMOTE_PACKAGE" -C ~/robot_ws/src
chmod +x ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/*.py ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/*.sh 2>/dev/null || true

cd ~/robot_ws
catkin_make
source ~/robot_ws/devel/setup.bash
python -m py_compile \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/judge_bridge.py \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/judge_smoke_test.py \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/fake_judge_server.py \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/board2_ocr_node.py \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/qrcode_voice_node.py \
  ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/patrol_mission.py

echo "DEPLOY_OK"
echo "Package: $(rospack find ros1_smart_pharmacy_patrol)"
'''


def connect(host, user, password, source_ip):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sock = None
    if source_ip:
        sock = socket.create_connection((host, 22), timeout=10, source_address=(source_ip, 0))
    else:
        sock = socket.create_connection((host, 22), timeout=10)
    client.connect(
        host,
        username=user,
        password=password,
        sock=sock,
        timeout=10,
        banner_timeout=10,
        auth_timeout=10,
        look_for_keys=False,
        allow_agent=False,
    )
    return client


def main():
    parser = argparse.ArgumentParser(description='Deploy judge-system-ready ROS1 package to EPRobot.')
    parser.add_argument('--host', default='192.168.12.1')
    parser.add_argument('--user', default='EPRobot')
    parser.add_argument('--password', default=os.environ.get('EPROBOT_PASSWORD', ''))
    parser.add_argument('--source-ip', default='192.168.12.248')
    parser.add_argument('--package', default=str(DEFAULT_PACKAGE))
    args = parser.parse_args()

    package = Path(args.package)
    if not package.is_file():
        print('package not found: %s' % package, file=sys.stderr)
        return 2

    password = args.password or getpass.getpass('Robot SSH password: ')
    print('connecting %s@%s...' % (args.user, args.host))
    client = connect(args.host, args.user, password, args.source_ip)
    try:
        remote_pkg = '/home/EPRobot/' + package.name
        print('uploading %s -> %s' % (package, remote_pkg))
        sftp = client.open_sftp()
        sftp.put(str(package), remote_pkg)
        sftp.close()

        print('extracting and building on robot...')
        stdin, stdout, stderr = client.exec_command('bash -s', timeout=240)
        stdin.write('export REMOTE_PACKAGE=%s\n' % shlex.quote(remote_pkg))
        stdin.write(REMOTE_SCRIPT)
        stdin.channel.shutdown_write()
        out = stdout.read().decode('utf-8', 'replace')
        err = stderr.read().decode('utf-8', 'replace')
        rc = stdout.channel.recv_exit_status()
        print(out)
        if err:
            print(err, file=sys.stderr)
        return rc
    finally:
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
