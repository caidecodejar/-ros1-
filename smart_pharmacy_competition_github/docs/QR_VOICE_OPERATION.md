# 二维码识别与语音播报操作说明

## 功能

`qrcode_voice_node.py` 用于识别识别板 1 上的二维码，并把识别结果同步到 ROS 话题和语音播报。

主要输出：

```text
/judge/cv1
/judge/task
/smart_pharmacy_patrol/status
/smart_pharmacy_patrol/board1_tasks
```

## 样本映射

```text
1 -> 静脉血样本 -> 血常规窗口
2 -> 唾液样本   -> 体液窗口
3 -> 组织样本   -> 免疫检测窗口
4 -> 血浆样本   -> 激素检验窗口
```

识别板 1 的二维码内容可表示为类似：

```text
1:A
2:B
3:C
AB:1
```

实际解析逻辑见：

```text
ros1_smart_pharmacy_patrol/scripts/patrol_mission.py
ros1_smart_pharmacy_patrol/scripts/qrcode_voice_node.py
```

## 启动

先启动相机：

```bash
roslaunch astra_camera astra.launch \
  depth_registration:=false \
  depth_processing:=false \
  depth_registered_processing:=false \
  ir_processing:=false
```

只启动二维码识别和语音：

```bash
roslaunch ros1_smart_pharmacy_patrol qrcode_voice.launch
```

比赛推荐启动方式：

```bash
START_BOARD2_OCR=false \
START_AUTO_ARRIVAL_DETECTOR=true \
INITIAL_CV2=FREE \
bash ~/robot_ws/src/ros1_smart_pharmacy_patrol/scripts/run_manual_competition.sh
```

## 检查

```bash
rostopic echo /judge/cv1
rostopic echo /smart_pharmacy_patrol/status
rostopic echo /smart_pharmacy_patrol/board1_tasks
```

如果二维码识别不稳定，可临时手动发布识别结果：

```bash
rostopic pub -1 /judge/cv1 std_msgs/String "data: '2:A'"
```

含义：A 窗口有唾液样本，应送到 2 号体液窗口。

## 到点播报

手动触发到达 A/B/C 取样窗口：

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: 'A'"
```

手动触发到达 1/2/3/4 化验窗口：

```bash
rostopic pub -1 /smart_pharmacy_patrol/manual_arrival std_msgs/String "data: '2'"
```

如果启用 `auto_arrival_detector.py`，小车停入窗口附近后会自动发布同一个话题。
