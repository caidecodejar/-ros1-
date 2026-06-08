# Judge Software Connection Guide

Date: 2026-06-04

## 2026-06-05 Update

The ROS1 judge bridge has now been implemented in:

```text
ros1_smart_pharmacy_patrol/scripts/judge_bridge.py
```

It sends newline-delimited UTF-8 JSON to the judge software TCP server:

```json
{"task":"A","speed":0.2,"odom":[-0.668,1.259],"CV1":"AB:1","CV2":"FREE"}
```

It also publishes the exact outgoing JSON on:

```text
/judge_bridge/payload
```

The current match-ready operation guide is:

```text
docs/judge_system_match_ready_20260605.md
```

The earlier section below saying the robot code did not yet implement the judge protocol is kept as historical context from 2026-06-04; it is no longer the current status.

## What This Software Does

The window shown by the user is the competition judge/scoring software.

Visible functions:

- Generates board/task information at match start.
- Shows a 4-minute countdown and current score.
- Displays the competition map, board areas, pickup windows, lab windows, and start area.
- Tracks main task statistics:
  - one delivery to 3 targets;
  - one delivery to 2 targets;
  - one delivery to 1 target;
  - board-1 recognition correctness.
- Tracks extra communication scoring:
  - voice broadcast success;
  - board/CV1 valid information;
  - board/CV2 cross-check;
  - task matching;
  - coordinate checking;
  - speed compliance.
- Tracks violations:
  - collision;
  - partial out-of-bounds;
  - complete out-of-bounds.

The screenshot also shows this error:

```text
The bound address is already in use
```

This means the judge software failed to bind/listen on its configured address/port, usually because another process or another instance of the judge software is already using that port.

## Current Likely Network Values

When the PC is connected to the robot WiFi, the values should usually be:

```text
Robot IP:         192.168.12.1
Judge software IP: 192.168.12.248
Port:             8888
```

The judge software should use the PC-side IP that the robot can reach. Do not use an unrelated campus/network IP such as `10.x.x.x` unless the robot is also on that network.

## Judge Software Fields

Recommended values:

```text
端口（默认8888）: 8888
裁判软件IP:       192.168.12.248
小车IP地址:       192.168.12.1
```

Then use:

- `端口切换`: restart/switch the judge server port.
- `开始比赛`: start the match countdown/task generation.
- `重置`: reset scoring and current match state.
- `识别板1`: likely opens or displays the first recognition board/task information.
- `小车信息屏`: likely displays received robot status information.

## Important Limitation In Current Robot Code

The current ROS1 mission code can publish task status on:

```text
/smart_pharmacy_patrol/status
```

But the current robot code does not yet implement the official judge communication protocol.

So filling in IP/port in the judge software is not enough by itself. The robot also needs a judge communication node that reads ROS data and sends it to the judge software.

Required robot-side data sources:

- `/cmd_vel` or chassis feedback speed.
- `/odom`.
- `/tf` or `map -> base_footprint` localization.
- `/smart_pharmacy_patrol/status`.
- board/QR recognition results.

These must be encoded according to the official judge protocol. If the protocol is not provided, it must be obtained from the competition group/software documentation or inferred from a sample client.

## Confirmed Static Findings From The Local Judge Software Executable

No cracking, patching, unpacking, or license bypass was performed. The executable was inspected read-only.

Confirmed implementation details:

- The judge software is a Qt/C++ Windows GUI program.
- It imports `Qt5Network.dll` and uses `QTcpServer` / `QTcpSocket`.
- It uses `QJsonDocument::fromJson`, `QJsonObject`, and `QJsonArray` for received data.
- The PC runs the TCP server. The robot is the TCP client.
- Default server port is `8888`.
- The software validates the connecting robot IP against the UI field `小车IP地址`.
- If the source IP does not match, the software logs an illegal-IP warning and disconnects the socket.
- The receive path appears to read line-based messages, so each JSON message should be terminated with `\n`.
- Plain-text field/type strings present in the executable include:

```text
task
speed
odom
CV1
CV2
CRASH
```

Inferred payload semantics from the executable:

- `speed`: numeric speed in m/s. The UI displays `当前车速: %1 m/s`.
- `odom`: JSON array, where `odom[0]` is current robot `x` and `odom[1]` is current robot `y`.
- `task`: string current task/location point, likely one of `A`, `B`, `C`, `1`, `2`, `3`, `4`.
- `CV1`: string from board 1 / QR recognition.
- `CV2`: string from board 2 / cross-check recognition.
- `CRASH`: collision event. The executable appears to support a plain text message beginning with `CRASH`.

Candidate minimal newline-delimited JSON message:

```json
{"task":"A","speed":0.08,"odom":[1.20,2.30],"CV1":"AB:1","CV2":"1"}
```

Send it over TCP as UTF-8 plus a trailing newline:

```text
{"task":"A","speed":0.08,"odom":[1.20,2.30],"CV1":"AB:1","CV2":"1"}\n
```

This format is inferred from clear strings and call flow in the executable. It still needs live validation with the running judge software because the official detailed protocol was not present in the local rules document.

## Fast Connection Test

On the PC:

```powershell
ping -S 192.168.12.248 -n 2 192.168.12.1
```

On the robot:

```bash
ping 192.168.12.248
```

If the judge server is listening on port `8888`, the robot should be able to connect:

```bash
python - <<'PY'
import socket
s = socket.create_connection(('192.168.12.248', 8888), timeout=3)
print('connected')
s.close()
PY
```

If this fails:

- confirm the judge software IP is `192.168.12.248`;
- confirm the port is `8888`;
- allow the judge software through Windows Firewall;
- close duplicate judge software instances;
- use a different port only if the robot-side client is changed to the same port.

## Fixing `The bound address is already in use`

On Windows PowerShell:

```powershell
netstat -ano | findstr ":8888"
```

If a process is shown, find it:

```powershell
Get-Process -Id <PID>
```

Stop it only if it is a stale judge/duplicate process:

```powershell
Stop-Process -Id <PID> -Force
```

Then restart the judge software or click `端口切换`.

If port `8888` is still blocked, use a different port such as `8890`, but the robot-side communication node must use the same port.

## Recommended Integration Plan

1. First make the judge software server start without the bind error.
2. Confirm PC and robot can ping each other.
3. Confirm the robot can open a TCP connection to `192.168.12.248:8888`.
4. Obtain the official protocol.
5. Add a ROS1 judge bridge node on the robot.
6. The judge bridge node should subscribe to ROS status topics and send speed, odometry, position, task status, and recognition results to the judge software.

Under time pressure, autonomous navigation should be tested first. Judge communication can be added after navigation is stable, because it is an extra scoring channel, while navigation and task completion are the core run.
