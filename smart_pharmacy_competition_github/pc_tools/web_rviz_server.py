#!/usr/bin/env python
from __future__ import annotations

import base64
import json
import math
import mimetypes
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import paramiko
import yaml
from PIL import Image


ROBOT_HOST = os.environ.get("EPROBOT_HOST", "192.168.12.1")
ROBOT_USER = os.environ.get("EPROBOT_USER", "EPRobot")
ROBOT_PASSWORD = os.environ.get("EPROBOT_PASSWORD", "")

HOST = os.environ.get("WEB_RVIZ_HOST", "127.0.0.1")
PORT = int(os.environ.get("WEB_RVIZ_PORT", "8765"))

ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parent
MAP_IMAGE = REPO_ROOT / "maps" / "competition" / "compitation_real_3p8x4p9.pgm"
ROUTE_YAML = REPO_ROOT / "ros1_smart_pharmacy_patrol" / "config" / "competition_navigation_route.yaml"
PLANNED_YAML = REPO_ROOT / "ros1_smart_pharmacy_patrol" / "config" / "competition_navigation_planned_paths.yaml"

STATE_LOCK = threading.RLock()
ROBOT_STATE = {
    "connected": False,
    "last_update": 0.0,
    "error": "not connected",
    "pose": None,
    "odom": None,
    "scan_points": [],
    "global_plan": [],
    "local_plan": [],
    "status": [],
    "cmd_speed": 0.0,
    "route_status": "",
}


REMOTE_COLLECTOR = r'''#!/usr/bin/env python
from __future__ import print_function

import json
import math
import sys
import threading

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    import tf
except Exception:
    tf = None


LOCK = threading.RLock()
STATE = {
    "pose": None,
    "odom": None,
    "scan": None,
    "global_plan": [],
    "local_plan": [],
    "status": [],
    "cmd_speed": 0.0,
    "route_status": "",
}


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def on_odom(msg):
    pose = msg.pose.pose
    yaw = quat_to_yaw(pose.orientation)
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    with LOCK:
        STATE["odom"] = {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw,
            "speed": math.sqrt(vx * vx + vy * vy),
        }


def on_cmd(msg):
    with LOCK:
        STATE["cmd_speed"] = msg.linear.x


def on_scan(msg):
    ranges = []
    total = len(msg.ranges)
    step = max(1, int(total / 180))
    angle = msg.angle_min
    for i, dist in enumerate(msg.ranges):
        if i % step:
            angle += msg.angle_increment
            continue
        if dist >= msg.range_min and dist <= msg.range_max and not math.isinf(dist) and not math.isnan(dist):
            ranges.append([angle, dist])
        angle += msg.angle_increment
    with LOCK:
        STATE["scan"] = ranges


def path_to_points(msg):
    points = []
    for pose_stamped in msg.poses[:500]:
        p = pose_stamped.pose.position
        points.append([p.x, p.y])
    return points


def on_global_plan(msg):
    with LOCK:
        STATE["global_plan"] = path_to_points(msg)


def on_local_plan(msg):
    with LOCK:
        STATE["local_plan"] = path_to_points(msg)


def on_status(msg):
    names = {
        0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED",
        4: "ABORTED", 5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING",
        8: "RECALLED", 9: "LOST",
    }
    values = []
    for item in msg.status_list[-6:]:
        values.append({"status": names.get(item.status, str(item.status)), "text": item.text})
    with LOCK:
        STATE["status"] = values


def on_route_status(msg):
    with LOCK:
        STATE["route_status"] = msg.data


def make_scan_points(pose, scan):
    if not pose or not scan:
        return []
    x = pose["x"]
    y = pose["y"]
    yaw = pose["yaw"]
    points = []
    for angle, dist in scan:
        a = yaw + angle
        points.append([x + math.cos(a) * dist, y + math.sin(a) * dist])
    return points


def main():
    rospy.init_node("codex_web_rviz_collector", anonymous=True)
    listener = tf.TransformListener() if tf is not None else None

    rospy.Subscriber("/odom", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/odometry/filtered", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/cmd_vel", Twist, on_cmd, queue_size=5)
    rospy.Subscriber("/scan_filtered", LaserScan, on_scan, queue_size=2)
    rospy.Subscriber("/scan", LaserScan, on_scan, queue_size=2)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, on_global_plan, queue_size=2)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, on_global_plan, queue_size=2)
    rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, on_local_plan, queue_size=2)
    rospy.Subscriber("/move_base/status", GoalStatusArray, on_status, queue_size=5)
    rospy.Subscriber("/codex_route_runner/status", String, on_route_status, queue_size=10)

    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        pose = None
        if listener is not None:
            try:
                trans, rot = listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                pose = {"x": trans[0], "y": trans[1], "yaw": yaw}
            except Exception:
                pass

        with LOCK:
            if pose is not None:
                STATE["pose"] = pose
            elif STATE["odom"] is not None:
                STATE["pose"] = dict(STATE["odom"])
            payload = {
                "pose": STATE["pose"],
                "odom": STATE["odom"],
                "scan_points": make_scan_points(STATE["pose"], STATE["scan"]),
                "global_plan": STATE["global_plan"],
                "local_plan": STATE["local_plan"],
                "status": STATE["status"],
                "cmd_speed": STATE["cmd_speed"],
                "route_status": STATE["route_status"],
            }
        sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        rate.sleep()


if __name__ == "__main__":
    main()
'''


REMOTE_ROUTE_RUNNER = r'''#!/usr/bin/env python
from __future__ import print_function

import math
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from std_msgs.msg import String
import yaml


ROUTES = {
    "default_AB_to_lab1": ["start", "board_1", "window_A", "window_B", "board_2", "lab_window_1", "start"],
    "sampled_manual_loop": ["start", "board_1", "window_C", "board_2", "lab_window_1", "lab_window_2", "lab_window_3", "lab_window_4", "start"],
    "all_targets_coverage": ["start", "board_1", "window_A", "window_B", "window_C", "board_2", "lab_window_1", "lab_window_2", "lab_window_3", "lab_window_4", "start"],
}

STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def make_goal(point):
    theta = float(point["theta"])
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(point["x"])
    goal.target_pose.pose.position.y = float(point["y"])
    goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)
    return goal


def main():
    route_name = sys.argv[1] if len(sys.argv) > 1 else "sampled_manual_loop"
    if route_name not in ROUTES:
        raise RuntimeError("unknown route: " + route_name)

    rospy.init_node("codex_route_runner")
    status_pub = rospy.Publisher("/codex_route_runner/status", String, queue_size=10, latch=True)
    waypoint_path = rospy.get_param("~waypoints", "/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml")
    timeout = float(rospy.get_param("~timeout", 120.0))

    with open(waypoint_path, "r") as handle:
        waypoints = (yaml.safe_load(handle) or {}).get("waypoints", {})

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    status_pub.publish("waiting_for_move_base")
    if not client.wait_for_server(rospy.Duration(30.0)):
        raise RuntimeError("move_base action server is not available")

    seq = ROUTES[route_name]
    status_pub.publish("started:%s:%s" % (route_name, ",".join(seq)))
    print("started", route_name, seq)

    try:
        for index, name in enumerate(seq):
            if rospy.is_shutdown():
                break
            if name not in waypoints:
                raise RuntimeError("missing waypoint: " + name)
            point = waypoints[name]
            status_pub.publish("goal:%d/%d:%s" % (index + 1, len(seq), name))
            print("goal", index + 1, len(seq), name, point)
            client.send_goal(make_goal(point))
            finished = client.wait_for_result(rospy.Duration(timeout))
            if not finished:
                client.cancel_goal()
                status_pub.publish("timeout:%s" % name)
                raise RuntimeError("timeout at " + name)
            state = client.get_state()
            state_name = STATUS_NAMES.get(state, str(state))
            status_pub.publish("result:%s:%s" % (name, state_name))
            print("result", name, state_name, client.get_goal_status_text())
            if state != GoalStatus.SUCCEEDED:
                raise RuntimeError("goal failed: %s state=%s" % (name, state_name))
            rospy.sleep(0.5)
        status_pub.publish("complete:%s" % route_name)
        print("complete", route_name)
    except Exception as exc:
        client.cancel_goal()
        status_pub.publish("failed:%s" % exc)
        raise


if __name__ == "__main__":
    main()
'''


REMOTE_START_LOOP = r'''#!/usr/bin/env bash
set +e
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME

route="${1:-sampled_manual_loop}"
mkdir -p /tmp/codex_logs
rosnode kill /codex_route_runner >/dev/null 2>&1 || true
rosnode list 2>/dev/null | grep keyboard_cmd_vel | xargs -r rosnode kill >/dev/null 2>&1 || true
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}" >/dev/null 2>&1 || true
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
tmux kill-session -t codex_route_loop >/dev/null 2>&1 || true
tmux new-session -d -s codex_route_loop "bash -lc 'source ~/eprobot_env.sh; source ~/robot_ws/devel/setup.bash; export ROBOT_TYPE=EPRobotV2.2; export ROS_MASTER_URI=http://192.168.12.1:11311; export ROS_IP=192.168.12.1; unset ROS_HOSTNAME; python /tmp/codex_route_runner.py \"$route\" > /tmp/codex_logs/route_loop.log 2>&1'"
echo "started route $route"
'''


REMOTE_STOP = r'''#!/usr/bin/env bash
set +e
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME

rosnode kill /codex_route_runner >/dev/null 2>&1 || true
rosnode kill /smart_pharmacy_patrol >/dev/null 2>&1 || true
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}" >/dev/null 2>&1 || true
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
tmux kill-session -t codex_route_loop >/dev/null 2>&1 || true
echo stopped
'''


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8-sig") as handle:
        return yaml.safe_load(handle) or {}


def load_config():
    route_data = load_yaml(ROUTE_YAML)
    planned_data = load_yaml(PLANNED_YAML)
    with Image.open(MAP_IMAGE) as image:
        width, height = image.size
    map_meta = route_data.get("map", {})
    return {
        "map": {
            "resolution": float(map_meta.get("resolution", 0.016)),
            "origin": map_meta.get("origin", [-2.147851, -1.060776, 0.0]),
            "width": width,
            "height": height,
        },
        "waypoints": route_data.get("waypoints", {}),
        "route_sets": route_data.get("route_sets", {}),
        "planned_routes": planned_data.get("planned_routes", {}),
    }


CONFIG = load_config()


def make_map_png() -> bytes:
    with Image.open(MAP_IMAGE) as image:
        image = image.convert("RGB")
        output = Path(os.environ.get("TEMP", str(ROOT))) / "web_rviz_map.png"
        image.save(output, "PNG")
        return output.read_bytes()


MAP_PNG = make_map_png()


def ssh_connect():
    if not ROBOT_PASSWORD:
        raise RuntimeError("Set EPROBOT_PASSWORD before using the Web RViz robot controls.")
    last_exc = None
    for _ in range(4):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                ROBOT_HOST,
                username=ROBOT_USER,
                password=ROBOT_PASSWORD,
                timeout=10,
                banner_timeout=10,
                auth_timeout=10,
                look_for_keys=False,
                allow_agent=False,
            )
            return client
        except Exception as exc:
            last_exc = exc
            try:
                client.close()
            except Exception:
                pass
            time.sleep(1.0)
    raise last_exc


def install_remote_scripts():
    client = ssh_connect()
    try:
        sftp = client.open_sftp()
        files = {
            "/tmp/codex_web_rviz_collector.py": REMOTE_COLLECTOR,
            "/tmp/codex_route_runner.py": REMOTE_ROUTE_RUNNER,
            "/tmp/codex_start_loop.sh": REMOTE_START_LOOP,
            "/tmp/codex_stop_motion.sh": REMOTE_STOP,
        }
        for remote_path, body in files.items():
            with sftp.file(remote_path, "w") as handle:
                handle.write(body)
            sftp.chmod(remote_path, 0o755)
        sftp.close()
    finally:
        client.close()


def exec_robot(command: str, timeout: int = 25):
    client = ssh_connect()
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        return out, err
    finally:
        client.close()


def collector_loop():
    while True:
        client = None
        try:
            client = ssh_connect()
            cmd = (
                "bash -lc 'source ~/eprobot_env.sh; "
                "source ~/robot_ws/devel/setup.bash; "
                "export ROBOT_TYPE=EPRobotV2.2; "
                "export ROS_MASTER_URI=http://192.168.12.1:11311; "
                "export ROS_IP=192.168.12.1; "
                "unset ROS_HOSTNAME; "
                "mkdir -p /tmp/codex_logs; "
                "python /tmp/codex_web_rviz_collector.py "
                "2>/tmp/codex_logs/web_rviz_collector.err'"
            )
            stdin, stdout, stderr = client.exec_command(cmd, timeout=None)
            with STATE_LOCK:
                ROBOT_STATE["connected"] = True
                ROBOT_STATE["error"] = ""
            for raw in iter(stdout.readline, ""):
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    data = json.loads(raw)
                except Exception:
                    continue
                with STATE_LOCK:
                    ROBOT_STATE.update(data)
                    ROBOT_STATE["connected"] = True
                    ROBOT_STATE["last_update"] = time.time()
                    ROBOT_STATE["error"] = ""
        except Exception as exc:
            with STATE_LOCK:
                ROBOT_STATE["connected"] = False
                ROBOT_STATE["error"] = str(exc)
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
        time.sleep(2.0)


HTML = r'''<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Browser RViz - EPRobot</title>
  <style>
    :root { color-scheme: dark; font-family: Segoe UI, Arial, sans-serif; }
    body { margin: 0; background: #15171a; color: #e8ecef; overflow: hidden; }
    #layout { display: grid; grid-template-columns: 1fr 330px; height: 100vh; }
    #viewWrap { position: relative; background: #0c0f12; }
    canvas { width: 100%; height: 100%; display: block; }
    #side { border-left: 1px solid #313840; padding: 14px; background: #1d2126; overflow: auto; }
    h1 { font-size: 18px; font-weight: 650; margin: 0 0 12px; }
    h2 { font-size: 13px; font-weight: 650; margin: 18px 0 8px; color: #aeb7c2; text-transform: uppercase; }
    .row { display: flex; gap: 8px; margin-bottom: 8px; }
    button, select { background: #2b333c; color: #e8ecef; border: 1px solid #46515d; border-radius: 6px; padding: 9px 10px; font-size: 13px; }
    button { cursor: pointer; }
    button.primary { background: #227a4e; border-color: #31a56c; }
    button.stop { background: #8d2f2f; border-color: #bd4747; }
    button:disabled { opacity: 0.5; cursor: default; }
    select { flex: 1; }
    .kv { display: grid; grid-template-columns: 92px 1fr; gap: 4px 10px; font-size: 13px; line-height: 1.45; }
    .label { color: #9aa5af; }
    .value { color: #f1f5f8; overflow-wrap: anywhere; }
    #log { white-space: pre-wrap; font-family: Consolas, monospace; font-size: 12px; color: #c6d0d8; background: #111417; border: 1px solid #323a43; border-radius: 6px; padding: 8px; min-height: 90px; }
    #badge { position: absolute; left: 12px; top: 12px; padding: 6px 9px; border-radius: 6px; background: rgba(0,0,0,.65); font-size: 13px; }
    .ok { color: #73e096; }
    .bad { color: #ff8c8c; }
  </style>
</head>
<body>
  <div id="layout">
    <div id="viewWrap">
      <canvas id="view"></canvas>
      <div id="badge">Connecting...</div>
    </div>
    <aside id="side">
      <h1>Browser RViz - EPRobot</h1>
      <div class="row">
        <select id="route">
          <option value="sampled_manual_loop">sampled_manual_loop</option>
          <option value="default_AB_to_lab1">default_AB_to_lab1</option>
          <option value="all_targets_coverage">all_targets_coverage</option>
        </select>
      </div>
      <div class="row">
        <button class="primary" id="startBtn">Start Route</button>
        <button class="stop" id="stopBtn">Stop</button>
      </div>
      <h2>Status</h2>
      <div class="kv">
        <div class="label">Robot</div><div class="value" id="conn">-</div>
        <div class="label">Pose</div><div class="value" id="pose">-</div>
        <div class="label">Speed</div><div class="value" id="speed">-</div>
        <div class="label">Route</div><div class="value" id="routeStatus">-</div>
        <div class="label">move_base</div><div class="value" id="mb">-</div>
      </div>
      <h2>Legend</h2>
      <div class="kv">
        <div class="label">Red</div><div class="value">robot pose</div>
        <div class="label">Green</div><div class="value">laser scan</div>
        <div class="label">Blue</div><div class="value">saved route</div>
        <div class="label">Yellow</div><div class="value">move_base global plan</div>
        <div class="label">Orange</div><div class="value">local plan</div>
      </div>
      <h2>Command Log</h2>
      <div id="log"></div>
    </aside>
  </div>
<script>
const cfg = __CONFIG__;
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const img = new Image();
img.src = '/map.png';
let state = null;
let scale = 1, ox = 0, oy = 0;

function worldToPixel(x, y) {
  const res = cfg.map.resolution;
  const origin = cfg.map.origin;
  const px = (x - origin[0]) / res;
  const py = cfg.map.height - ((y - origin[1]) / res);
  return [px * scale + ox, py * scale + oy];
}

function yawPoint(x, y, yaw, len) {
  return [x + Math.cos(yaw) * len, y + Math.sin(yaw) * len];
}

function resize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = Math.max(800, Math.floor(rect.width * devicePixelRatio));
  canvas.height = Math.max(600, Math.floor(rect.height * devicePixelRatio));
  const sx = canvas.width / cfg.map.width;
  const sy = canvas.height / cfg.map.height;
  scale = Math.min(sx, sy) * 0.94;
  ox = (canvas.width - cfg.map.width * scale) / 2;
  oy = (canvas.height - cfg.map.height * scale) / 2;
}

function drawPolyline(points, color, width) {
  if (!points || points.length < 2) return;
  ctx.beginPath();
  for (let i = 0; i < points.length; i++) {
    const p = worldToPixel(points[i][0], points[i][1]);
    if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function drawWaypoints() {
  ctx.font = `${12 * devicePixelRatio}px Segoe UI`;
  ctx.textBaseline = 'middle';
  for (const [name, p] of Object.entries(cfg.waypoints)) {
    const q = worldToPixel(p.x, p.y);
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(q[0], q[1], 4 * devicePixelRatio, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#101418';
    ctx.fillText(name, q[0] + 7 * devicePixelRatio, q[1]);
  }
}

function routePoints(routeName) {
  const route = cfg.planned_routes[routeName];
  const out = [];
  if (!route || !route.segments) return out;
  for (const segment of route.segments) {
    for (const p of segment.clean_path || []) out.push(p);
  }
  return out;
}

function drawRobot(pose) {
  if (!pose) return;
  const p = worldToPixel(pose.x, pose.y);
  const tip = worldToPixel(...yawPoint(pose.x, pose.y, pose.yaw, 0.18));
  const left = worldToPixel(...yawPoint(pose.x, pose.y, pose.yaw + 2.45, 0.11));
  const right = worldToPixel(...yawPoint(pose.x, pose.y, pose.yaw - 2.45, 0.11));
  ctx.fillStyle = '#ff4d4d';
  ctx.beginPath();
  ctx.moveTo(tip[0], tip[1]);
  ctx.lineTo(left[0], left[1]);
  ctx.lineTo(right[0], right[1]);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5 * devicePixelRatio;
  ctx.stroke();
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(p[0], p[1], 2.5 * devicePixelRatio, 0, Math.PI * 2);
  ctx.fill();
}

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#0c0f12';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (img.complete) ctx.drawImage(img, ox, oy, cfg.map.width * scale, cfg.map.height * scale);

  const routeName = document.getElementById('route').value;
  drawPolyline(routePoints(routeName), '#4aa3ff', 3 * devicePixelRatio);
  drawWaypoints();

  if (state) {
    drawPolyline(state.global_plan, '#ffd84a', 2.2 * devicePixelRatio);
    drawPolyline(state.local_plan, '#ff9a35', 2.2 * devicePixelRatio);
    if (state.scan_points) {
      ctx.fillStyle = '#42ff90';
      for (const s of state.scan_points) {
        const p = worldToPixel(s[0], s[1]);
        ctx.fillRect(p[0] - 1.2 * devicePixelRatio, p[1] - 1.2 * devicePixelRatio, 2.4 * devicePixelRatio, 2.4 * devicePixelRatio);
      }
    }
    drawRobot(state.pose);
  }
  requestAnimationFrame(render);
}

async function poll() {
  try {
    const r = await fetch('/state', {cache: 'no-store'});
    state = await r.json();
    const fresh = state.last_update && (Date.now() / 1000 - state.last_update < 2.0);
    document.getElementById('conn').innerHTML = state.connected && fresh ? '<span class="ok">connected</span>' : '<span class="bad">stale/offline</span>';
    document.getElementById('badge').innerHTML = state.connected && fresh ? '<span class="ok">ROS live</span>' : '<span class="bad">ROS stale</span>';
    if (state.pose) document.getElementById('pose').textContent = `${state.pose.x.toFixed(3)}, ${state.pose.y.toFixed(3)}, yaw ${state.pose.yaw.toFixed(2)}`;
    document.getElementById('speed').textContent = `${Number(state.cmd_speed || 0).toFixed(3)} m/s`;
    document.getElementById('routeStatus').textContent = state.route_status || '-';
    document.getElementById('mb').textContent = (state.status || []).map(s => s.status).join(', ') || '-';
  } catch (e) {
    document.getElementById('conn').innerHTML = '<span class="bad">viewer error</span>';
  }
}

async function post(path) {
  const log = document.getElementById('log');
  log.textContent = 'Running command...';
  const r = await fetch(path, {method: 'POST'});
  const data = await r.json();
  log.textContent = (data.ok ? 'OK\n' : 'ERROR\n') + (data.out || '') + (data.err || '');
}

document.getElementById('startBtn').onclick = () => {
  const route = encodeURIComponent(document.getElementById('route').value);
  post('/start?route=' + route);
};
document.getElementById('stopBtn').onclick = () => post('/stop');
window.onresize = resize;
resize();
setInterval(poll, 250);
render();
</script>
</body>
</html>
'''


class Handler(BaseHTTPRequestHandler):
    def send_bytes(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, payload, code=200):
        self.send_bytes(code, json.dumps(payload, separators=(",", ":")).encode("utf-8"), "application/json")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            html = HTML.replace("__CONFIG__", json.dumps(CONFIG, separators=(",", ":")))
            self.send_bytes(200, html.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/map.png":
            self.send_bytes(200, MAP_PNG, "image/png")
            return
        if path == "/state":
            with STATE_LOCK:
                payload = dict(ROBOT_STATE)
            self.send_json(payload)
            return
        self.send_json({"ok": False, "err": "not found"}, code=404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/start":
            route = parse_qs(parsed.query).get("route", ["sampled_manual_loop"])[0]
            if route not in CONFIG["route_sets"]:
                self.send_json({"ok": False, "err": "unknown route: " + route}, code=400)
                return
            try:
                out, err = exec_robot("bash /tmp/codex_start_loop.sh %s" % route, timeout=20)
                self.send_json({"ok": True, "out": out, "err": err})
            except Exception as exc:
                self.send_json({"ok": False, "err": str(exc)}, code=500)
            return
        if parsed.path == "/stop":
            try:
                out, err = exec_robot("bash /tmp/codex_stop_motion.sh", timeout=20)
                self.send_json({"ok": True, "out": out, "err": err})
            except Exception as exc:
                self.send_json({"ok": False, "err": str(exc)}, code=500)
            return
        self.send_json({"ok": False, "err": "not found"}, code=404)

    def log_message(self, fmt, *args):
        sys.stdout.write("%s - %s\n" % (self.address_string(), fmt % args))
        sys.stdout.flush()


def main():
    print("Installing remote helper scripts on %s..." % ROBOT_HOST, flush=True)
    install_remote_scripts()
    print("Starting collector thread...", flush=True)
    threading.Thread(target=collector_loop, daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print("Browser RViz running at http://%s:%d/" % (HOST, PORT), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
