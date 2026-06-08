from __future__ import annotations

import hashlib
import html
import os
import shutil
import time
import zipfile
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
SOURCE_DIR = WORKSPACE / "ros1_smart_pharmacy_patrol"
DEPLOY_ARCHIVE = WORKSPACE / "ros1_smart_pharmacy_patrol_deploy_20260604_123943.tar.gz"
DEPLOY_MANIFEST = WORKSPACE / "ros1_smart_pharmacy_patrol_manifest_20260604_123943.txt"


REPORT_LINES = [
    "# 智慧医疗送检小车技术报告",
    "",
    "## 摘要",
    "",
    "本作品面向“智慧药房/智慧医疗送检”比赛任务，设计并实现一套基于 ROS1 的阿克曼结构竞速小车自主运行系统。系统以 Ubuntu 18.04 + ROS1 Melodic 为运行环境，依托激光雷达、里程计、TF 定位、move_base 导航栈、任务调度节点和裁判通信节点，实现从起点出发、到识别板获取任务、前往体检窗口取样、经过识别板二判断化验区状态、到指定化验窗口配送并返回起点的流程。当前提交包重点包含 ROS1 任务调度、真实采样航点、裁判 TCP JSON 通信桥、部署脚本和运行说明。",
    "",
    "系统的核心思路是：底层由小车平台完成电机与舵机控制，上层 ROS 节点负责地图定位、路径规划、任务状态机与裁判系统通信。自主导航采用静态栅格地图与实测航点，结合 move_base 全局规划和局部规划完成路径跟踪；任务调度采用有限状态流程，根据识别板一结果决定 A/B/C 取样点，根据样本类型映射到 1/2/3/4 化验窗口；裁判通信由小车作为 TCP 客户端主动连接电脑端裁判软件，周期性上传任务、速度、里程计位置和识别结果。",
    "",
    "## 一、技术方案设计与作品技术梳理",
    "",
    "### 1.1 总体控制思路",
    "",
    "作品采用分层控制架构。最底层是 EPRobot 小车底盘与控制板，负责舵机、电机、编码器等实时控制；中间层是 ROS1 驱动与导航环境，提供 `/cmd_vel`、`/odom`、`/scan`、`/scan_filtered`、`/map`、`/tf`、`/move_base` 等标准接口；上层是智慧药房任务节点和裁判通信节点，负责任务逻辑、航点派发、状态发布和局域网上传。",
    "",
    "系统运行链路如下：",
    "",
    "1. 地图服务器加载比赛场地图，地图尺寸按实测 3.8m × 4.9m 配置，地图分辨率为 0.016m。",
    "2. 激光雷达发布原始扫描，滤波节点生成 `/scan_filtered`，定位节点输出 `map -> base_footprint`。",
    "3. `move_base` 接收上层任务节点发送的目标点，计算全局路径和局部速度指令。",
    "4. 底盘控制节点接收 `/cmd_vel`，转换为阿克曼底盘可执行的速度和转向控制量。",
    "5. `patrol_mission.py` 根据识别结果决定航点序列，并发布 `/smart_pharmacy_patrol/status`。",
    "6. `judge_bridge.py` 订阅速度、里程计、TF 和任务状态，将数据按 JSON 行协议发送至裁判软件。",
    "",
    "### 1.2 自主导航功能实现",
    "",
    "自主导航依赖 ROS1 导航栈。小车启动底层导航后，系统中应存在 `/map_server`、`/move_base`、`/laser_filter`、`/talos_laser_loc`、`/ekf_se` 等节点。地图坐标系以 `map` 为全局坐标，车体坐标以 `base_footprint` 表示。任务节点把每个比赛点位封装成 `move_base_msgs/MoveBaseGoal`，目标姿态采用二维 yaw 角转换为四元数。",
    "",
    "本项目已采样的真实航点保存在 `config/waypoints_real.yaml`，主要包括：",
    "",
    "- `start`：起点。",
    "- `board_1`：识别板一。",
    "- `window_A`、`window_B`、`window_C`：体检区三个取样窗口。",
    "- `board_2`：识别板二。",
    "- `lab_window_1` 至 `lab_window_4`：化验区四个窗口。",
    "",
    "任务运行时，`patrol_mission.py` 按顺序调用 `go(waypoint_name)`，在到达一个目标后短暂停留，用于满足规则中“车身进入方框并有明显停留”的要求。若 `move_base` 返回失败或超时，节点会取消当前目标并发布失败状态，便于调试。",
    "",
    "### 1.3 人机交互功能实现",
    "",
    "人机交互分为调试交互和比赛交互两类。调试阶段通过 SSH、RViz/noVNC、键盘控制节点和 ROS 命令行观察小车状态；比赛阶段通过裁判软件、语音播报和任务状态上传体现人机交互。当前代码已提供任务状态话题 `/smart_pharmacy_patrol/status` 和裁判通信话题 `/judge/task`、`/judge/cv1`、`/judge/cv2`，可在视觉识别未完全稳定时手动写入识别结果进行联调。",
    "",
    "裁判软件连接方案为：电脑端裁判软件作为 TCP Server，小车端 `judge_bridge.py` 作为 TCP Client，连接 `192.168.12.248:8888`，周期性发送 UTF-8 JSON 行。该方案能避免修改裁判软件本体，符合只读分析得到的通信方向和字段要求。",
    "",
    "### 1.4 任务调度功能实现",
    "",
    "任务调度采用有限状态流程。默认流程为：起点 -> 识别板一 -> 体检窗口 A/B/C 中的目标窗口 -> 识别板二 -> 对应化验窗口 -> 返回起点。识别板一的结果格式可用 `AB:1`、`ABC:2`、`C:4` 表示，其中前半部分表示需要取样的体检窗口，后半部分表示目标化验窗口或样本类别；识别板二结果可用 `FREE` 或 `BUSY:5` 表示化验区空闲或需等待秒数。",
    "",
    "样本与化验窗口映射关系为：",
    "",
    "- 静脉血样本 -> 1 号血常规窗口。",
    "- 唾液样本 -> 2 号体液窗口。",
    "- 组织样本 -> 3 号免疫检测窗口。",
    "- 血浆样本 -> 4 号激素检验窗口。",
    "",
    "代码中的 `parse_board_1()` 负责解析识别板一信息，`parse_board_2()` 负责解析识别板二信息，`run_once()` 负责执行完整任务流程。",
    "",
    "## 二、专业关键技术的实现思路",
    "",
    "### 2.1 ROS1 软件架构",
    "",
    "ROS1 包名为 `ros1_smart_pharmacy_patrol`，主要代码文件如下：",
    "",
    "- `scripts/patrol_mission.py`：智慧药房任务调度节点，负责解析任务、发送导航目标、发布任务状态。",
    "- `scripts/judge_bridge.py`：裁判通信桥，负责采集 ROS 数据并通过 TCP JSON 上报。",
    "- `launch/patrol.launch`：启动巡航任务节点。",
    "- `launch/judge_bridge.launch`：启动裁判通信节点。",
    "- `launch/competition.launch`：比赛组合启动文件，默认只开裁判通信，不让车运动；需要真实导航时显式设置 `start_patrol:=true send_goals:=true`。",
    "- `config/waypoints_real.yaml`：实测比赛航点。",
    "- `config/competition_navigation_route.yaml`：点位稳定性、路线顺序和地图参数说明。",
    "",
    "### 2.2 定位与传感器融合",
    "",
    "系统使用激光雷达作为主要环境感知传感器，通过 `/scan` 和 `/scan_filtered` 提供障碍物距离信息。定位链路使用地图坐标系下的 `map -> base_footprint` 变换作为主要位姿来源，里程计和滤波结果可通过 `/odom`、`/odometry/filtered` 读取。`judge_bridge.py` 优先使用 TF 中的 `map -> base_footprint` 更新当前位置，如果 TF 暂时不可用，则退回使用里程计话题中的位置数据。",
    "",
    "### 2.3 裁判通信关键技术",
    "",
    "裁判软件需要小车实时上传速度、里程计、视觉识别结果、所在位置和当前任务。为此新增 `judge_bridge.py`，它订阅以下数据源：",
    "",
    "- `/cmd_vel`：用于估计当前速度。",
    "- `/odom` 与 `/odometry/filtered`：用于获取里程计与速度反馈。",
    "- TF `map -> base_footprint`：用于获取地图坐标系下的位置。",
    "- `/smart_pharmacy_patrol/status`：用于解析当前任务阶段。",
    "- `/judge/task`、`/judge/cv1`、`/judge/cv2`：用于手动或视觉节点写入裁判字段。",
    "",
    "上报 JSON 示例：",
    "",
    "```json",
    "{\"task\":\"A\",\"speed\":0.08,\"odom\":[1.20,2.30],\"CV1\":\"AB:1\",\"CV2\":\"FREE\"}",
    "```",
    "",
    "每条消息以换行符结尾，便于裁判软件按行读取。节点支持断线重连，若 Windows 防火墙阻挡 `8888` 端口，会在日志中提示连接超时，待端口放行后自动重连。",
    "",
    "### 2.4 工程可行性",
    "",
    "该方案的可行性体现在三个方面：第一，使用 ROS1 标准接口与现有小车平台兼容，不需要重写底盘驱动；第二，任务调度与导航解耦，可以在 dry-run 模式下验证任务逻辑，也可以在真实模式下发送 `move_base` 目标；第三，裁判通信独立于导航节点，即使导航任务未启动，也可以先验证裁判软件连接和状态上传。",
    "",
    "## 三、单片机驱动方法、底盘控制模型和控制算法",
    "",
    "### 3.1 底盘结构与参数",
    "",
    "小车采用阿克曼结构底盘，前轮转向、后轮驱动。根据实测参数，车体主框架长度约 0.271m，宽度约 0.210m，轴距约 0.30m，轮距约 0.185m，最大转向角约 0.49rad（约 28°）。雷达安装在车体前部上方，作为定位与避障传感器。",
    "",
    "### 3.2 单片机/底层驱动方法",
    "",
    "上层 ROS 不直接控制电机 PWM，而是向底盘驱动节点发布标准速度指令 `/cmd_vel`。底层驱动节点由 EPRobot 平台提供，将线速度和角速度转换为电机速度与舵机转角，再由控制板输出到执行机构。这样可以把实时性要求较高的电机闭环和舵机控制留在底层，把路径规划和任务逻辑放在 ROS 上层。",
    "",
    "控制链路为：",
    "",
    "```text",
    "move_base 局部规划器 -> /cmd_vel -> 底盘驱动节点 -> 控制板/单片机 -> 电机和舵机",
    "```",
    "",
    "### 3.3 阿克曼运动模型",
    "",
    "阿克曼底盘不能像差速车一样原地旋转，控制算法需要考虑最小转弯半径。理想模型中，线速度为 v，前轮转角为 δ，轴距为 L，则角速度满足：",
    "",
    "```text",
    "ω = v * tan(δ) / L",
    "```",
    "",
    "当局部规划器输出目标线速度和角速度时，底层会根据轴距和转向限制换算舵机角度。为了避免路径中出现无法执行的急转弯，需要在局部规划参数中限制最大角速度、最大转向角和速度变化率，并在地图上保留足够通行空间。",
    "",
    "### 3.4 控制算法与安全策略",
    "",
    "比赛运行时推荐使用较低速度进行定位与任务点停靠，建图和航点采样时速度建议为 0.08-0.15m/s。正式巡航时可根据场地稳定性逐步提高速度，但进入窗口方框、识别板区域和狭窄区域时应降低速度，避免碰撞挡板或因定位跳变导致规划失败。",
    "",
    "系统通过 `move_base` 状态反馈判断目标是否成功到达。若目标失败，任务节点会发布 `navigation_failed` 状态并取消目标，避免继续盲目发送后续目标。现场调试时可用 `/move_base/cancel` 和 `/cmd_vel` 零速度命令紧急停车。",
    "",
    "## 四、计算机视觉识别原理、方案及代码实现",
    "",
    "### 4.1 识别任务分析",
    "",
    "比赛视觉任务包括识别板一和识别板二。识别板一通过二维码给出体检区 A/B/C 中哪些窗口有样本，以及对应样本类别或目标化验窗口；识别板二给出化验区空闲或忙碌状态，若忙碌还需读取等待时间。识别结果直接影响小车后续航点序列和停留动作。",
    "",
    "### 4.2 视觉识别方案",
    "",
    "推荐视觉流程为：相机采集图像 -> 图像去畸变/裁剪 -> 灰度化和自适应阈值 -> 二维码检测与解码 -> 多帧投票滤波 -> 发布 ROS 识别结果。二维码识别可使用 OpenCV QRCodeDetector、zbar 或 pyzbar；识别板二若为文字提示，可采用模板匹配或 OCR，也可以将提示内容编码为二维码以提高稳定性。",
    "",
    "为了提升比赛稳定性，识别节点不应只采一帧，而应连续读取多帧结果，只有当同一结果连续出现或投票占比超过阈值时才发布。发布格式建议使用 `std_msgs/String`，例如识别板一发布 `AB:1`，识别板二发布 `FREE` 或 `BUSY:8`。",
    "",
    "### 4.3 当前代码接口实现",
    "",
    "当前 ROS1 任务代码已经预留识别结果接口。`patrol.launch` 提供 `board1` 和 `board2` 参数，用于在视觉节点未完全稳定时模拟识别结果；`judge_bridge.py` 提供 `/judge/cv1` 和 `/judge/cv2` 话题，用于把真实识别结果或手动测试结果上传裁判软件。后续将视觉节点接入后，可把识别结果同时写入任务节点和裁判通信节点。",
    "",
    "示例命令：",
    "",
    "```bash",
    "rostopic pub -1 /judge/cv1 std_msgs/String \"data: 'AB:1'\"",
    "rostopic pub -1 /judge/cv2 std_msgs/String \"data: 'FREE'\"",
    "```",
    "",
    "### 4.4 视觉部分的改进方向",
    "",
    "当前提交包的主线是 ROS1 导航、任务调度和裁判通信。视觉识别接口已经预留，但真实相机识别仍需现场继续确认相机驱动、图像话题和二维码识别稳定性。若比赛时间紧张，可优先保证导航路线、裁判通信和手动写入识别结果联调稳定；若相机恢复正常，再接入真实视觉节点完成自动识别。",
    "",
    "## 五、路径规划算法方案和技术实现",
    "",
    "### 5.1 地图构建与比例标定",
    "",
    "比赛场地按 3.8m × 4.9m 实测尺寸进行地图比例标定，地图分辨率设置为 0.016m。通过键盘控制小车沿安全区域运行并记录激光定位轨迹，结合 RViz 保存地图和点位采样结果，得到可用于自主导航的静态地图与真实航点。",
    "",
    "### 5.2 全局路径规划",
    "",
    "全局规划由 `move_base` 根据静态栅格地图完成。上层任务节点只指定目标点，实际路径由全局规划器在可通行区域内计算。为便于报告和调试，`competition_navigation_planned_paths.yaml` 中保存了离线参考路线和点位顺序，但真实运行仍以 `move_base` 在当前地图和代价地图上计算的路径为准。",
    "",
    "### 5.3 局部路径规划与避障",
    "",
    "局部规划器根据激光雷达数据、局部代价地图和阿克曼运动约束生成速度指令。小车在狭窄区域需要降低速度，避免局部规划给出无法执行的转向。调试中若出现 “Failed to find a valid control” 一类错误，通常说明当前位置、障碍物膨胀半径、机器人轮廓或航点目标姿态不适配，需要重新定位、调整局部规划参数或重新采样航点。",
    "",
    "### 5.4 航点路线设计",
    "",
    "当前真实采样航点包括起点、识别板、三个体检窗口和四个化验窗口。默认比赛示例 `board1:=AB:1 board2:=FREE` 的路线为：",
    "",
    "```text",
    "start -> board_1 -> window_A -> window_B -> board_2 -> lab_window_1 -> start",
    "```",
    "",
    "若识别板一给出其他窗口组合，任务节点会按识别结果依次前往对应体检窗口，再根据样本类型前往对应化验窗口。所有航点均以地图坐标形式保存，包含 `x`、`y` 和 `theta`。",
    "",
    "### 5.5 技术实现效果与风险控制",
    "",
    "当前系统已经具备 ROS1 导航、真实航点配置、任务流程调度和裁判通信桥功能。主要风险在于现场定位稳定性、相机识别稳定性和 Windows 防火墙对裁判端口的拦截。比赛前应完成三项检查：第一，RViz 中小车位姿与真实起点一致；第二，小车能从 `board_1` 到各窗口安全到达并停入方框；第三，小车端到电脑裁判软件 `192.168.12.248:8888` 连接状态为 `ESTABLISHED`。",
    "",
    "## 六、部署与测试说明",
    "",
    "小车端环境变量：",
    "",
    "```bash",
    "source ~/eprobot_env.sh",
    "source ~/robot_ws/devel/setup.bash",
    "export ROBOT_TYPE=EPRobotV2.2",
    "export ROS_MASTER_URI=http://192.168.12.1:11311",
    "export ROS_IP=192.168.12.1",
    "unset ROS_HOSTNAME",
    "```",
    "",
    "只启动裁判通信：",
    "",
    "```bash",
    "roslaunch ros1_smart_pharmacy_patrol judge_bridge.launch judge_host:=192.168.12.248 judge_port:=8888 initial_task:=start initial_cv1:=AB:1 initial_cv2:=FREE",
    "```",
    "",
    "启动真实点位自主巡航：",
    "",
    "```bash",
    "roslaunch ros1_smart_pharmacy_patrol patrol.launch send_goals:=true waypoints:=/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml board1:=AB:1 board2:=FREE",
    "```",
    "",
    "裁判软件推荐填写：",
    "",
    "```text",
    "端口: 8888",
    "裁判软件IP: 192.168.12.248",
    "小车IP地址: 192.168.12.1",
    "```",
    "",
    "若小车连接裁判软件超时，需要在管理员 PowerShell 中放行端口：",
    "",
    "```powershell",
    "New-NetFirewallRule -DisplayName \"Smart Pharmacy Judge 8888\" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8888 -Profile Any",
    "```",
    "",
    "## 七、结论",
    "",
    "本作品以 ROS1 为核心实现了智慧医疗送检小车的软件框架，完成了自主导航任务调度、实测航点管理、裁判系统通信桥和比赛运行脚本整理。该方案结构清晰、模块解耦，能够在现有 EPRobot 小车平台上继续扩展真实视觉识别和语音播报功能。后续重点是完成相机驱动稳定性验证、二维码/OCR 识别节点接入、局部规划参数微调和正式比赛视频录制。",
]


def ignore_source_files(_dirpath: str, names: list[str]) -> list[str]:
    ignored = []
    for name in names:
        if name in {".git", "__pycache__"} or name.endswith((".pyc", ".pyo", ".log", ".tmp")):
            ignored.append(name)
    return ignored


def paragraph_xml(text: str, style: str | None = None) -> str:
    text = text.rstrip("\n")
    if text == "":
        return "<w:p/>"
    ppr = f'<w:pPr><w:pStyle w:val="{style}"/></w:pPr>' if style else ""
    return f'<w:p>{ppr}<w:r><w:t xml:space="preserve">{html.escape(text)}</w:t></w:r></w:p>'


def code_para_xml(text: str) -> str:
    return (
        '<w:p><w:pPr><w:pStyle w:val="Code"/></w:pPr><w:r><w:rPr>'
        '<w:rFonts w:ascii="Consolas" w:eastAsia="Consolas"/><w:sz w:val="20"/>'
        f'</w:rPr><w:t xml:space="preserve">{html.escape(text)}</w:t></w:r></w:p>'
    )


def write_docx(path: Path, title: str, lines: list[str]) -> None:
    body = []
    in_code = False
    for line in lines:
        if line.startswith("```"):
            in_code = not in_code
            continue
        if in_code:
            body.append(code_para_xml(line))
        elif line.startswith("# "):
            body.append(paragraph_xml(line[2:], "Title"))
        elif line.startswith("## "):
            body.append(paragraph_xml(line[3:], "Heading1"))
        elif line.startswith("### "):
            body.append(paragraph_xml(line[4:], "Heading2"))
        elif line.startswith("- "):
            body.append(paragraph_xml("• " + line[2:], "ListParagraph"))
        elif line and line[0].isdigit() and ". " in line[:4]:
            body.append(paragraph_xml(line, "ListParagraph"))
        else:
            body.append(paragraph_xml(line))

    document_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><w:body>'
        + "\n".join(body)
        + '<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1440" w:right="1440" '
        'w:bottom="1440" w:left="1440" w:header="708" w:footer="708" w:gutter="0"/></w:sectPr>'
        '</w:body></w:document>'
    )
    styles_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">'
        '<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/>'
        '<w:rPr><w:rFonts w:ascii="Times New Roman" w:eastAsia="宋体"/><w:sz w:val="24"/></w:rPr>'
        '<w:pPr><w:spacing w:line="360" w:lineRule="auto"/></w:pPr></w:style>'
        '<w:style w:type="paragraph" w:styleId="Title"><w:name w:val="Title"/><w:basedOn w:val="Normal"/>'
        '<w:pPr><w:jc w:val="center"/><w:spacing w:after="360"/></w:pPr>'
        '<w:rPr><w:rFonts w:ascii="Times New Roman" w:eastAsia="黑体"/><w:b/><w:sz w:val="36"/></w:rPr></w:style>'
        '<w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:basedOn w:val="Normal"/>'
        '<w:pPr><w:spacing w:before="360" w:after="180"/></w:pPr>'
        '<w:rPr><w:rFonts w:ascii="Times New Roman" w:eastAsia="黑体"/><w:b/><w:sz w:val="30"/></w:rPr></w:style>'
        '<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/>'
        '<w:pPr><w:spacing w:before="240" w:after="120"/></w:pPr>'
        '<w:rPr><w:rFonts w:ascii="Times New Roman" w:eastAsia="黑体"/><w:b/><w:sz w:val="26"/></w:rPr></w:style>'
        '<w:style w:type="paragraph" w:styleId="ListParagraph"><w:name w:val="List Paragraph"/>'
        '<w:basedOn w:val="Normal"/><w:pPr><w:ind w:left="420"/></w:pPr></w:style>'
        '<w:style w:type="paragraph" w:styleId="Code"><w:name w:val="Code"/><w:basedOn w:val="Normal"/>'
        '<w:rPr><w:rFonts w:ascii="Consolas" w:eastAsia="Consolas"/><w:sz w:val="20"/></w:rPr>'
        '<w:pPr><w:spacing w:before="60" w:after="60"/></w:pPr></w:style></w:styles>'
    )
    content_types = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
        '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
        '<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
        '<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
        '</Types>'
    )
    rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>'
        '</Relationships>'
    )
    word_rels = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>'
        '</Relationships>'
    )
    core = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/">'
        f"<dc:title>{html.escape(title)}</dc:title><dc:creator>smart_hospital_ws</dc:creator>"
        "</cp:coreProperties>"
    )
    app = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
        'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
        "<Application>Microsoft Word</Application></Properties>"
    )
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", content_types)
        archive.writestr("_rels/.rels", rels)
        archive.writestr("word/_rels/document.xml.rels", word_rels)
        archive.writestr("word/document.xml", document_xml)
        archive.writestr("word/styles.xml", styles_xml)
        archive.writestr("docProps/core.xml", core)
        archive.writestr("docProps/app.xml", app)


def main() -> None:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    root_name = f"smart_pharmacy_ros1_submission_{stamp}"
    submit_root = WORKSPACE / root_name
    if submit_root.exists():
        shutil.rmtree(submit_root)

    report_dir = submit_root / "01_report"
    source_out = submit_root / "02_source_code"
    deploy_dir = submit_root / "03_deployment_package"
    run_doc_dir = submit_root / "04_run_instructions"
    video_dir = submit_root / "05_video_materials_todo"
    extra_docs_dir = submit_root / "06_extra_docs"
    for directory in [report_dir, source_out, deploy_dir, run_doc_dir, video_dir, extra_docs_dir]:
        directory.mkdir(parents=True, exist_ok=True)

    shutil.copytree(SOURCE_DIR, source_out / "ros1_smart_pharmacy_patrol", ignore=ignore_source_files)
    if DEPLOY_ARCHIVE.exists():
        shutil.copy2(DEPLOY_ARCHIVE, deploy_dir / DEPLOY_ARCHIVE.name)
    if DEPLOY_MANIFEST.exists():
        shutil.copy2(DEPLOY_MANIFEST, deploy_dir / DEPLOY_MANIFEST.name)

    report_md = "\n".join(REPORT_LINES) + "\n"
    (report_dir / "technical_report_smart_pharmacy_car.md").write_text(report_md, encoding="utf-8")
    write_docx(report_dir / "technical_report_smart_pharmacy_car.docx", "智慧医疗送检小车技术报告", REPORT_LINES)

    submission_readme = f"""# 提交包说明

本目录按比赛技术报告要求整理，包含技术报告、ROS1 源代码、部署包、运行说明和视频占位目录。

## 目录结构

```text
{root_name}/
  01_report/
    technical_report_smart_pharmacy_car.docx
    technical_report_smart_pharmacy_car.md
  02_source_code/
    ros1_smart_pharmacy_patrol/
  03_deployment_package/
    ros1_smart_pharmacy_patrol_deploy_20260604_123943.tar.gz
    ros1_smart_pharmacy_patrol_manifest_20260604_123943.txt
  04_run_instructions/
    README_run_and_submit.md
    code_file_summary.md
  05_video_materials_todo/
    put_competition_video_here.txt
  06_extra_docs/
    judge_software_connection.md
```

## 报告要求对应关系

1. 技术方案设计、作品技术梳理、自主导航、人机交互、任务调度：见技术报告第一章。
2. 专业关键技术实现思路：见技术报告第二章。
3. 单片机驱动方法、底盘控制模型和控制算法：见技术报告第三章。
4. 计算机视觉识别原理、具体方案及代码实现：见技术报告第四章。
5. 路径规划算法方案和技术实现：见技术报告第五章。

## 正式提交前建议

- 将最终 zip 按比赛要求重命名为“学校_队伍编号.zip”。
- 将比赛录制视频放入 `05_video_materials_todo/` 后再提交。
- 如老师要求只交报告和代码，可保留 `01_report`、`02_source_code`、`03_deployment_package`。
"""
    (run_doc_dir / "README_run_and_submit.md").write_text(submission_readme, encoding="utf-8")

    code_summary = """# 代码文件作用说明

## ros1_smart_pharmacy_patrol/scripts/patrol_mission.py

ROS1 智慧药房任务调度节点。主要功能：读取航点 YAML；解析识别板一结果；解析识别板二状态；向 `move_base` 发送导航目标；发布 `/smart_pharmacy_patrol/status` 任务状态。

关键函数：

- `parse_board_1(payload)`: 解析如 `AB:1`、`ABC:2` 的识别板一结果。
- `parse_board_2(payload)`: 解析 `FREE` 或 `BUSY:8` 的识别板二结果。
- `make_goal(point)`: 将航点转换为 `MoveBaseGoal`。
- `go(waypoint_name)`: 发送导航目标并等待结果。
- `run_once()`: 执行完整比赛任务流程。

## ros1_smart_pharmacy_patrol/scripts/judge_bridge.py

ROS1 到裁判软件的 TCP JSON 通信桥。主要功能：订阅 `/cmd_vel`、`/odom`、`/odometry/filtered`、TF、`/smart_pharmacy_patrol/status`；生成 `task/speed/odom/CV1/CV2` JSON；小车端主动连接电脑裁判软件 `192.168.12.248:8888`；断线后自动重连。

## launch 文件

- `patrol.launch`: 启动任务调度节点。
- `judge_bridge.launch`: 启动裁判通信节点。
- `competition.launch`: 组合启动文件，默认只启动裁判通信，不让小车运动。

## config 文件

- `waypoints_real.yaml`: 实测比赛点位。
- `competition_navigation_route.yaml`: 点位稳定性和路线说明。
- `competition_navigation_planned_paths.yaml`: 离线路径参考，不替代运行时 move_base 规划。
"""
    (run_doc_dir / "code_file_summary.md").write_text(code_summary, encoding="utf-8")

    video_note = """请将正式比赛录制视频放入本目录后再提交。

建议视频内容包含：
1. 小车从起点出发。
2. 到识别板一读取/确认任务。
3. 到 A/B/C 体检窗口取样并停入方框。
4. 到识别板二识别化验区状态。
5. 到 1/2/3/4 化验窗口配送并停入方框。
6. 裁判软件或 RViz 状态画面。
"""
    (video_dir / "put_competition_video_here.txt").write_text(video_note, encoding="utf-8")

    judge_doc = """# 裁判软件连接说明

电脑连接小车 WiFi 后，推荐填写：

```text
端口: 8888
裁判软件IP: 192.168.12.248
小车IP地址: 192.168.12.1
```

通信方向：电脑裁判软件是 TCP Server，小车 `judge_bridge.py` 是 TCP Client。

小车端启动：

```bash
roslaunch ros1_smart_pharmacy_patrol judge_bridge.launch judge_host:=192.168.12.248 judge_port:=8888 initial_task:=start initial_cv1:=AB:1 initial_cv2:=FREE
```

检查连接：

```bash
rosnode list | grep judge_bridge
rostopic echo -n 1 /judge_bridge/status
ss -tnp | grep ':8888'
```

若连接超时，通常是 Windows 防火墙拦截小车 WiFi 网段入站连接。管理员 PowerShell 执行：

```powershell
New-NetFirewallRule -DisplayName "Smart Pharmacy Judge 8888" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8888 -Profile Any
```
"""
    (extra_docs_dir / "judge_software_connection.md").write_text(judge_doc, encoding="utf-8")

    manifest_lines = [f"submission: {root_name}", f"generated: {stamp}", ""]
    for path in sorted(submit_root.rglob("*")):
        if path.is_file():
            data = path.read_bytes()
            rel = path.relative_to(submit_root).as_posix()
            manifest_lines.append(f"{rel}\t{len(data)}\t{hashlib.sha1(data).hexdigest()}")
    (submit_root / "submission_manifest.txt").write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")

    zip_path = WORKSPACE / f"{root_name}.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(submit_root.rglob("*")):
            if path.is_file():
                arcname = Path(root_name) / path.relative_to(submit_root)
                archive.write(path, arcname.as_posix())

    print(f"submit_root={submit_root}")
    print(f"zip={zip_path}")
    print(f"zip_size={zip_path.stat().st_size}")
    print(f"docx={report_dir / 'technical_report_smart_pharmacy_car.docx'}")
    print(f"files={sum(1 for path in submit_root.rglob('*') if path.is_file())}")


if __name__ == "__main__":
    main()
