# 识别板1/识别板2比赛规则接入说明

更新日期：2026-06-05

## 当前结论

原来的 ROS1 代码不完全符合你补充的比赛规则。旧逻辑主要按固定参数 `board1:=AB:1`、`board2:=FREE` 运行：

- 识别板1只理解“二维码内容”，没有根据二维码在识别板上的第 1/2/3/4 位置决定化验窗口。
- 识别板2没有 ROS1 实时 OCR 节点，不能自动识别“窗口2空闲/忙碌/等待8秒”并播报。
- 裁判桥可以发 `CV1/CV2`，但旧 `CV1` 格式没有承载多二维码位置任务。

本次已改成比赛规则版本：

- 识别板1：相机识别多个二维码，并按画面 2x2 区域映射到 1/2/3/4 化验窗口。
- 识别板1任务格式：`1:A,3:C` 表示“1号二维码/化验窗口1需要去 A 取样，3号二维码/化验窗口3需要去 C 取样”。
- 识别板1二维码内容：`A`/`B`/`C`/`AB`/`BC`/`ABC` 表示要去哪个体检窗口取样。
- 识别板2：新增 OCR 节点，识别 `FREE`/`BUSY`、`空闲`/`忙碌`/`等待` 和等待秒数。
- 识别板2如果识别到 `窗口2 忙碌 等待8秒`，会发布 `status=busy;wait=8;window=2` 并播报“识别到化验窗口2忙碌，等待8秒”。
- 识别板2如果识别到空闲，会发布 `FREE`，任务节点最多停留 0.5 秒后继续，满足“三秒内通过”的要求。
- 如果识别板2明确写了窗口编号，任务节点默认只在该窗口属于本轮目标化验窗口时应用等待；如果要无条件按识别板2等待，可启动时加 `board2_apply_only_matching_window:=false`。

## 修改的代码

```text
ros1_smart_pharmacy_patrol/scripts/qrcode_voice_node.py
ros1_smart_pharmacy_patrol/scripts/board2_ocr_node.py
ros1_smart_pharmacy_patrol/scripts/patrol_mission.py
ros1_smart_pharmacy_patrol/scripts/judge_bridge.py
ros1_smart_pharmacy_patrol/launch/qrcode_voice.launch
ros1_smart_pharmacy_patrol/launch/board2_ocr.launch
ros1_smart_pharmacy_patrol/launch/patrol.launch
ros1_smart_pharmacy_patrol/launch/competition.launch
tools/deploy_judge_system_to_robot.py
```

## 识别板1位置规则

默认按相机画面 2x2 分区：

```text
左上 = 1    右上 = 2
左下 = 3    右下 = 4
```

如果现场识别板编号顺序不是这个顺序，可以通过 `qr_slot_order` 改。例如如果现场是：

```text
左上 = 1    右上 = 3
左下 = 2    右下 = 4
```

启动时用：

```bash
qr_slot_order:=1,3,2,4
```

## 识别板1发布的话题

二维码节点会发布：

```text
/smart_pharmacy_patrol/board1_tasks    1:A,3:C
/judge/cv1                             1:A,3:C
/smart_pharmacy_patrol/status          vision_board_1:tasks=1:A,3:C;selected_slot=1;qr=A;windows=A;target_lab=1
```

裁判桥会把 `/judge/cv1` 或 `/smart_pharmacy_patrol/status` 转成 JSON 的 `CV1` 字段。

示例 JSON：

```json
{"task":"board_1","speed":0.0,"odom":[-0.012,0.274],"CV1":"1:A,3:C","CV2":"FREE"}
```

## 识别板2发布的话题

OCR 节点会发布：

```text
/smart_pharmacy_patrol/board2_status   status=busy;wait=8;window=2;text=...
/judge/cv2                             BUSY:8
/smart_pharmacy_patrol/voice           识别到化验窗口2忙碌，等待8秒
/smart_pharmacy_patrol/status          vision_board_2:status=busy;wait=8;window=2;text=...
```

如果空闲：

```text
/smart_pharmacy_patrol/board2_status   status=free;wait=0;window=2;text=...
/judge/cv2                             FREE
```

## 正式比赛推荐启动方式

先确认小车已部署新压缩包、相机话题存在、裁判软件已打开并监听 `192.168.12.248:8888`。

只启动裁判桥和视觉识别，不让小车动：

```bash
roslaunch ros1_smart_pharmacy_patrol competition.launch \
  judge_host:=192.168.12.248 \
  judge_port:=8888 \
  judge_source_ip:=192.168.12.1 \
  start_judge_bridge:=true \
  start_qrcode_voice:=true \
  start_board2_ocr:=true \
  start_patrol:=false \
  send_goals:=false
```

观察识别结果：

```bash
rostopic echo /smart_pharmacy_patrol/board1_tasks
rostopic echo /smart_pharmacy_patrol/board2_status
rostopic echo /judge_bridge/payload
ls -l /home/EPRobot/vision_test/
```

确认识别稳定后，再启动自主任务：

```bash
roslaunch ros1_smart_pharmacy_patrol competition.launch \
  judge_host:=192.168.12.248 \
  judge_port:=8888 \
  judge_source_ip:=192.168.12.1 \
  start_judge_bridge:=true \
  start_qrcode_voice:=true \
  start_board2_ocr:=true \
  start_patrol:=true \
  send_goals:=true \
  use_live_board1:=true \
  use_live_board2:=true \
  execute_all_board1_tasks:=true \
  max_board1_tasks:=2 \
  waypoints:=/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml
```

## 手动兜底方式

如果相机或 OCR 不稳定，可以用手动参数先跑通裁判和导航。

例如识别板1当前两个二维码是：

```text
1号二维码内容 A
3号二维码内容 C
```

识别板2显示窗口2忙碌等待8秒：

```bash
roslaunch ros1_smart_pharmacy_patrol competition.launch \
  judge_host:=192.168.12.248 \
  judge_port:=8888 \
  judge_source_ip:=192.168.12.1 \
  start_judge_bridge:=true \
  start_qrcode_voice:=false \
  start_board2_ocr:=false \
  start_patrol:=true \
  send_goals:=true \
  use_live_board1:=false \
  use_live_board2:=false \
  board1:=1:A,3:C \
  board2:=window=2;status=busy;wait=8 \
  execute_all_board1_tasks:=true \
  max_board1_tasks:=2 \
  waypoints:=/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml
```

## 必须上车验证的点

本地已通过 Python 语法检查和离线逻辑测试，但电脑端没有 ROS 环境，以下内容必须在小车上验证：

1. `/camera/rgb/image_rect_color` 或 `/camera/rgb/image_raw` 是否存在。
2. `pyzbar` 是否能在现场距离识别二维码。
3. `pytesseract` 和 tesseract 中文/英文语言包是否装在小车系统里。
4. 识别板1实际编号顺序是否等于默认 `1,2,3,4`。
5. 识别板2 OCR 对现场字体是否稳定，必要时需要调整 `roi_x1/roi_y1/roi_x2/roi_y2` 裁剪区域。
6. 裁判软件是否接受 `CV1="1:A,3:C"` 这种多二维码格式；如果裁判只接受单任务格式，可以临时把 `max_board1_tasks:=1`。

## 当前风险

导航、裁判桥、二维码位置解析和 OCR 解析逻辑已经补齐；最大风险仍是现场视觉稳定性，尤其是识别板2文字 OCR。比赛时间紧时，建议先用“只开视觉不跑车”的命令验证识别结果，再决定是否开启 `use_live_board1/use_live_board2`。如果 OCR 不稳定，使用手动 `board2:=window=2;status=busy;wait=8` 兜底更稳。
