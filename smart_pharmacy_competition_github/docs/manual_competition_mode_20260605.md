# ROS1 manual competition mode

This is the conservative race mode for the current hardware state.

## What it changes

- Keyboard teleop is the only node allowed to command `/cmd_vel`.
- QR recognition for board 1 keeps running and publishes `/judge/cv1`.
- Board 2 OCR can run in parallel and publishes `/judge/cv2`; if nothing is posted, the initial value remains `FREE`.
- Manual arrival voice can be triggered by `/smart_pharmacy_patrol/manual_arrival`.
- `judge_bridge` keeps sending speed, odom, task, `CV1`, and `CV2` to the judge software.
- `patrol_mission` is not started.
- No `/move_base` goal is sent.

## Why this is safer

The previous failure happened during autonomous navigation to `window_A`.
Manual mode avoids local-planner aborts by letting the driver control the chassis while keeping recognition and judge synchronization alive.

## Start order

Start the judge software on the PC first:

```text
Port: 8888
Judge software IP: 192.168.12.248
Robot IP: 192.168.12.1
```

Start the robot navigation/camera stack:

```bash
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME

roslaunch robot_navigation robot_race_init.launch \
  map_file:=/home/EPRobot/robot_ws/src/robot_navigation/maps/compitation_real_3p8x4p9.yaml \
  open_rviz:=false
```

Start recognition and judge bridge only:

```bash
bash ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/run_manual_competition.sh
```

To also enable automatic arrival detection:

```bash
START_AUTO_ARRIVAL_DETECTOR=true \
bash ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/run_manual_competition.sh
```

Start keyboard control in another terminal:

```bash
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Runtime checks

There should be only one active `/cmd_vel` publisher: the keyboard node.

```bash
rostopic info /cmd_vel
```

Recognition output:

```bash
rostopic echo /judge/cv1
rostopic echo /judge/cv2
rostopic echo /smart_pharmacy_patrol/status
```

Judge packets:

```bash
rostopic echo /judge_bridge/payload
```

## Manual arrival voice

Sample mapping from the rules:

```text
1 -> 静脉血样本 -> 血常规窗口
2 -> 唾液样本   -> 体液窗口
3 -> 组织样本   -> 免疫检测窗口
4 -> 血浆样本   -> 激素检验窗口
```

The node starts by default in `manual_competition.launch`.
It does not publish `/cmd_vel`.

It listens to board 1 QR results from:

```text
/smart_pharmacy_patrol/board1_tasks
/judge/cv1
```

When the robot is manually driven into a pickup window, trigger:

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'A'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'B'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'C'"
```

Example spoken text:

```text
到达A窗口，取到A窗口中的唾液样本
```

When the robot is manually driven into a lab window, trigger:

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '1'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '2'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '3'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '4'"
```

Equivalent lab names are also accepted:

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'lab1'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'lab_window_2'"
```

Example spoken text:

```text
到达体液窗口，送达唾液样本，样本数为1
```

Manual board 1 fallback for testing:

```bash
rostopic pub -1 /judge/cv1 std_msgs/String "data: '2:A'"
```

This means: A pickup window has saliva samples and should be delivered to lab window 2.

## Automatic arrival trigger

Automatic mode removes the need to publish `/smart_pharmacy_patrol/manual_arrival` by hand.

It uses the live `map -> base_footprint` pose and the ROS navigation waypoints from:

```text
/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml
```

Important:

```text
Use ROS waypoints for auto arrival detection.
Use judge coordinates only in the judge software.
These are different coordinate frames.
```

Default trigger gates:

```text
distance to pickup window <= 0.32 m
distance to lab window    <= 0.32 m
robot speed               <= 0.05 m/s
dwell time inside zone    >= 1.0 s
cooldown per point        >= 12 s
board 1 recognition       required before any auto trigger
```

Runtime status:

```bash
rostopic echo /smart_pharmacy_patrol/auto_arrival_status
```

If it triggers, it publishes the same topic as the manual command:

```text
/smart_pharmacy_patrol/manual_arrival
```

So the voice behavior is the same as manual trigger.

If the robot enters a window but does not trigger, stop the robot fully inside the box for at least 1 second.
If localization is visibly wrong in RViz, do not use automatic mode; use the manual trigger commands above.

## Fallbacks

If board 2 has no posted content:

```bash
rostopic pub -1 /judge/cv2 std_msgs/String "data: 'FREE'"
```

If board 1 recognition is unstable, manually publish the recognized result:

```bash
rostopic pub -1 /judge/cv1 std_msgs/String "data: '1:A'"
rostopic pub -1 /judge/task std_msgs/String "data: 'A'"
```

For this conservative mode, do not start:

```bash
roslaunch ros1_smart_pharmacy_patrol patrol.launch send_goals:=true
roslaunch ros1_smart_pharmacy_patrol competition.launch start_patrol:=true send_goals:=true
```
