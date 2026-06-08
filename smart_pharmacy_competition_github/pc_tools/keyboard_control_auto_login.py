import getpass
import os
import socket
import sys
import textwrap
import time

import paramiko

try:
    import msvcrt
except ImportError:  # pragma: no cover - this launcher is intended for Windows.
    msvcrt = None


HOST = "192.168.12.1"
USER = "EPRobot"
REMOTE_SCRIPT = "/tmp/codex_keyboard_default.sh"


REMOTE_SCRIPT_BODY = r"""#!/usr/bin/env bash
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash

export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME

COMPETITION_MAP_FILE="${COMPETITION_MAP_FILE:-/home/EPRobot/robot_ws/src/robot_navigation/maps/compitation_real_3p8x4p9.yaml}"
MAP_ARG=""
if [ -f "$COMPETITION_MAP_FILE" ]; then
  MAP_ARG="map_file:=$COMPETITION_MAP_FILE"
else
  echo "[warn] competition map not found: $COMPETITION_MAP_FILE; robot_race_init.launch will use its default map."
fi

mkdir -p /tmp/codex_logs

echo "[init] checking ROS stack and radar..."
if ! rosnode list >/dev/null 2>&1 || ! rosnode list 2>/dev/null | grep -qx /ls01d || { [ -n "$MAP_ARG" ] && ! ps -ef | grep -F "map_server $COMPETITION_MAP_FILE" | grep -v grep >/dev/null 2>&1; }; then
  echo "[init] starting robot_race_init.launch in tmux session codex_race_init..."
  if [ -n "$MAP_ARG" ]; then
    echo "[init] map: $COMPETITION_MAP_FILE"
  fi
  tmux kill-session -t codex_race_init 2>/dev/null || true
  tmux new-session -d -s codex_race_init "bash -lc 'source ~/eprobot_env.sh; source ~/robot_ws/devel/setup.bash; export ROBOT_TYPE=EPRobotV2.2; export ROS_MASTER_URI=http://192.168.12.1:11311; export ROS_IP=192.168.12.1; unset ROS_HOSTNAME; roslaunch robot_navigation robot_race_init.launch $MAP_ARG open_rviz:=false > /tmp/codex_logs/robot_race_init_tmux.log 2>&1'"

  for _ in $(seq 1 25); do
    if rosnode list 2>/dev/null | grep -qx /ls01d; then
      break
    fi
    sleep 1
  done
fi

echo "[init] stopping autonomous navigation and old keyboard control..."
rosnode list 2>/dev/null | grep keyboard_cmd_vel | xargs -r rosnode kill 2>/dev/null || true
rosnode kill /move_base 2>/dev/null || true
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true

if rosnode list 2>/dev/null | grep -qx /ls01d; then
  echo "[ready] radar node /ls01d is running."
else
  echo "[warn] radar node /ls01d was not detected. Check /tmp/codex_logs/robot_race_init_tmux.log on the robot."
fi

echo "[ready] keyboard control uses COMPETITION parameters: speed=0.25, turn=0.35, key_timeout=0.35."
echo "[keys] w/a/s/d move, Space stop, x exit, q/e speed, z/c turn."
echo ""

python ~/smart_pharmacy_patrol_runtime/keyboard_cmd_vel_mapping.py _speed:=0.25 _turn:=0.35 _key_timeout:=0.35
"""


def write_remote_script(client: paramiko.SSHClient) -> None:
    with client.open_sftp() as sftp:
        with sftp.file(REMOTE_SCRIPT, "w") as remote_file:
            remote_file.write(REMOTE_SCRIPT_BODY)
        sftp.chmod(REMOTE_SCRIPT, 0o755)


def print_banner() -> None:
    print("EPRobot keyboard control - auto login, speed 0.25 m/s")
    print("Robot: EPRobot@192.168.12.1")
    print("Keys: w/a/s/d move, Space stop, x exit, q/e speed, z/c turn")
    print("Keep clear of the robot before pressing movement keys.")
    print("")


def read_channel(channel: paramiko.Channel) -> None:
    while True:
        printed = False
        while channel.recv_ready():
            sys.stdout.buffer.write(channel.recv(4096))
            sys.stdout.buffer.flush()
            printed = True
        while channel.recv_stderr_ready():
            sys.stderr.buffer.write(channel.recv_stderr(4096))
            sys.stderr.buffer.flush()
            printed = True
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            return
        if not printed:
            time.sleep(0.02)


def keyboard_loop(channel: paramiko.Channel) -> None:
    if msvcrt is None:
        print("This launcher needs Windows msvcrt for direct key control.", file=sys.stderr)
        return

    print("[local] terminal is active. Press keys directly; use Space to stop, x to exit.")
    try:
        while not channel.exit_status_ready():
            if not msvcrt.kbhit():
                time.sleep(0.01)
                continue

            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                # Discard Windows extended-key suffix, such as arrow/function keys.
                if msvcrt.kbhit():
                    msvcrt.getwch()
                continue
            if key == "\r":
                key = "\n"
            if key == "\x03":
                channel.send("\x03")
                return
            channel.send(key.encode("utf-8", errors="ignore"))
    except KeyboardInterrupt:
        try:
            channel.send("\x03")
        except Exception:
            pass


def main() -> int:
    print_banner()
    password = os.environ.get("EPROBOT_PASSWORD") or getpass.getpass("Password: ")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print("[local] connecting...")
        client.connect(
            HOST,
            username=USER,
            password=password,
            timeout=12,
            banner_timeout=12,
            auth_timeout=12,
            look_for_keys=False,
            allow_agent=False,
        )
        write_remote_script(client)

        channel = client.get_transport().open_session()
        channel.get_pty(term="xterm", width=120, height=40)
        channel.exec_command(f"bash {REMOTE_SCRIPT}")

        import threading

        reader = threading.Thread(target=read_channel, args=(channel,), daemon=True)
        reader.start()
        keyboard_loop(channel)
        reader.join(timeout=2)

        if channel.exit_status_ready():
            return channel.recv_exit_status()
        return 0
    except (paramiko.SSHException, socket.error, TimeoutError) as exc:
        print(f"[error] SSH connection failed: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
