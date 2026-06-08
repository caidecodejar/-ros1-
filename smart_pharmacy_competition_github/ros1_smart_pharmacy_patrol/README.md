# ros1_smart_pharmacy_patrol

这是智慧药房比赛小车使用的 ROS1 catkin 功能包。

这个包主要提供：

- 裁判系统 TCP JSON 通信桥
- 识别板 1 二维码识别与语音播报
- 识别板 2 忙碌/空闲 OCR 识别
- 手动与自动到点语音触发
- 保守模式比赛启动文件
- 可选的 `move_base` 自主巡航任务

## 重要启动文件

```text
launch/manual_competition.launch      # 推荐的真实场地比赛模式
launch/competition.launch             # 裁判/二维码/OCR/巡航组合启动入口
launch/judge_bridge.launch            # 只启动裁判 TCP 通信桥
launch/qrcode_voice.launch            # 只启动识别板 1 二维码与语音
launch/board2_ocr.launch              # 只启动识别板 2 OCR
launch/manual_arrival_voice.launch    # 只启动到点播报节点
launch/auto_arrival_detector.launch   # 基于航点的自动到点触发
launch/patrol.launch                  # 可选的 move_base 自主巡航
```

## 主要脚本

```text
scripts/run_manual_competition.sh     # 推荐的小车端比赛辅助启动脚本
scripts/judge_bridge.py               # 向裁判软件发送 task/speed/odom/CV1/CV2
scripts/qrcode_voice_node.py          # 读取相机图像，识别识别板 1 二维码并播报
scripts/board2_ocr_node.py            # 识别识别板 2 的忙碌/空闲/等待状态
scripts/manual_arrival_voice_node.py  # 到达取样/化验窗口后播报取样和送样信息
scripts/auto_arrival_detector.py      # 根据 TF 位姿判断是否到达窗口并自动触发
scripts/patrol_mission.py             # 可选自主巡航任务，向 move_base 发送目标点
scripts/judge_smoke_test.py           # 对裁判软件做一次性 TCP JSON 测试
scripts/fake_judge_server.py          # 本地调试用的假裁判服务器
```

## 配置文件

```text
config/waypoints_real.yaml
config/competition_navigation_route.yaml
config/competition_navigation_planned_paths.yaml
```

`waypoints_real.yaml` 用于 ROS 导航和自动到点检测。裁判软件坐标只应在电脑端裁判软件中使用，不能直接当作 ROS 地图坐标使用，因为两者不在同一个坐标系。

## 推荐的真实场地运行方式

先单独启动小车底盘与定位栈：

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

再启动相机：

```bash
roslaunch astra_camera astra.launch \
  depth_registration:=false \
  depth_processing:=false \
  depth_registered_processing:=false \
  ir_processing:=false
```

再启动比赛辅助节点：

```bash
export JUDGE_HOST=192.168.12.248
export JUDGE_PORT=8888
export JUDGE_SOURCE_IP=192.168.12.1
export START_BOARD2_OCR=false
export START_AUTO_ARRIVAL_DETECTOR=true
export INITIAL_CV2=FREE

bash ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/run_manual_competition.sh
```

这个启动方式会开启识别、语音、裁判桥和可选自动到点检测，但不会主动发布 `/cmd_vel`。

## 运行检查

`/cmd_vel` 应该只有键盘节点一个发布者：

```bash
rostopic info /cmd_vel
```

预期订阅者：

```text
/base_control
/judge_bridge
```

常用检查话题：

```bash
rostopic echo /judge/cv1
rostopic echo /judge/cv2
rostopic echo /judge_bridge/payload
rostopic echo /smart_pharmacy_patrol/status
rostopic echo /smart_pharmacy_patrol/auto_arrival_status
```

如果自动到点不稳定，可以手动触发：

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'A'"
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '1'"
```

## 编译

```bash
cd ~/robot_ws
catkin_make
source ~/robot_ws/devel/setup.bash
```

`package.xml` 里声明的是本包依赖；相机、雷达、底盘控制和定位地图相关包由 EPRobot 原有环境提供。
