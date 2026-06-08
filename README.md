# 智慧药房 ROS1 竞赛车代码仓库

本仓库整理了智慧药房比赛小车当前使用的 ROS1 代码、电脑端辅助工具、地图与航点文件，以及比赛运行说明。

当前推荐的比赛策略是保守模式：

- 小车运动由键盘人工控制。
- 小车继续运行摄像头二维码识别、语音播报、到点判断和裁判系统 TCP 同步。
- 自主 `move_base` 巡航代码仍保留在仓库中，但只有在真实场地定位和路线稳定后才建议启用。

## 仓库结构

```text
.
|- ros1_smart_pharmacy_patrol/   # ROS1 catkin 比赛包，包含任务逻辑、裁判桥、二维码/OCR、语音、到点检测
|- pc_tools/                     # Windows/电脑端 SSH、键盘控制、部署、Web RViz 工具
|- tools/                        # 打包、路线重生成、小车部署工具
|- maps/competition/             # 比赛地图、采样航点、路线文件、裁判坐标 CSV
|- docs/                         # 比赛说明、规则核查、裁判软件连接、运行文档
|- requirements-pc.txt           # 电脑端 Python 依赖
|- .gitignore
`- .gitattributes
```

这个 GitHub 版本有意排除了以下内容：

- 裁判软件 `.exe`
- 小车真实 SSH 密码
- 日志、`__pycache__`、临时截图、安装包和历史压缩产物

## 硬件与网络假设

当前调试时使用的网络配置如下：

```text
小车用户名:       EPRobot
小车 IP:         192.168.12.1
电脑小车网卡 IP: 192.168.12.248
裁判 TCP 端口:   8888
地图大小:         3.8 m x 4.9 m
ROS 地图文件:     compitation_real_3p8x4p9.yaml
```

不要把小车 SSH 密码提交到仓库。电脑端脚本运行前，只在当前终端临时设置：

```powershell
$env:EPROBOT_PASSWORD = "<ROBOT_SSH_PASSWORD>"
```

## 小车端 ROS 环境

在小车上：

```bash
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME
```

先用比赛地图启动底盘与定位栈：

```bash
roslaunch robot_navigation robot_race_init.launch \
  map_file:=/home/EPRobot/robot_ws/src/robot_navigation/maps/compitation_real_3p8x4p9.yaml \
  open_rviz:=false
```

启动相机：

```bash
roslaunch astra_camera astra.launch \
  depth_registration:=false \
  depth_processing:=false \
  depth_registered_processing:=false \
  ir_processing:=false
```

启动比赛辅助节点：

```bash
export JUDGE_HOST=192.168.12.248
export JUDGE_PORT=8888
export JUDGE_SOURCE_IP=192.168.12.1
export START_BOARD2_OCR=false
export START_AUTO_ARRIVAL_DETECTOR=true
export INITIAL_CV2=FREE

bash ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/run_manual_competition.sh
```

在这个模式下，辅助节点不应发布 `/cmd_vel`，速度控制应只来自键盘节点。

## 电脑端键盘控制

在仓库根目录下的 Windows PowerShell 中执行：

```powershell
pip install -r requirements-pc.txt
$env:EPROBOT_PASSWORD = "<ROBOT_SSH_PASSWORD>"
powershell -ExecutionPolicy Bypass -File .\pc_tools\start_keyboard_control_auto_login.ps1
```

`pc_tools/keyboard_control_auto_login.py` 当前默认比赛参数：

```text
speed = 0.25 m/s
turn = 0.35
key_timeout = 0.35 s
```

控制按键：

```text
w: 前进
s: 后退
a: 左转
d: 右转
Space: 停车
x: 退出
q/e: 调整速度
z/c: 调整转向
```

实车启动前，先按一次空格，再轻按 `w` 试车。

## 裁判软件连接

裁判软件运行在电脑端，作为 TCP Server；小车端 `judge_bridge.py` 作为 TCP Client，持续发送 UTF-8 JSON 行数据。

当前会上传的字段包括：

```text
task
speed
odom
CV1
CV2
CRASH
```

裁判软件建议填写：

```text
端口:            8888
裁判软件 IP:     192.168.12.248
小车 IP 地址:    192.168.12.1
```

可在小车端做一次烟雾测试：

```bash
python3 ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/judge_smoke_test.py \
  --host 192.168.12.248 \
  --port 8888 \
  --task A \
  --speed 0.2 \
  --cv1 "AB:1" \
  --cv2 "FREE"
```

## 主要运行话题

```text
/cmd_vel                                  # 底盘速度指令，键盘节点应是唯一发布者
/judge/task                               # 当前任务/窗口
/judge/cv1                                # 识别板 1 二维码结果
/judge/cv2                                # 识别板 2 忙碌/空闲结果
/judge_bridge/payload                     # 发送给裁判软件的 JSON 数据
/smart_pharmacy_patrol/status             # 任务/识别状态
/smart_pharmacy_patrol/manual_arrival     # 手动或自动到点触发
/smart_pharmacy_patrol/auto_arrival_status
```

## 部署

将这个 ROS1 包放到小车：

```text
~/robot_ws/src/ros1_smart_pharmacy_patrol
```

然后编译：

```bash
cd ~/robot_ws
catkin_make
source ~/robot_ws/devel/setup.bash
```

如果已经生成部署压缩包，也可以使用 `tools/deploy_judge_system_to_robot.py` 部署到小车。

## 上传 GitHub

在当前目录执行：

```bash
git init
git add .
git commit -m "Initial smart pharmacy ROS1 competition code"
git branch -M main
git remote add origin <你的 GitHub 仓库地址>
git push -u origin main
```

推送前建议检查：

```bash
git status
git diff --cached --stat
```

确认没有把真实密码、裁判软件 `.exe`、本地日志和临时截图一起提交。
