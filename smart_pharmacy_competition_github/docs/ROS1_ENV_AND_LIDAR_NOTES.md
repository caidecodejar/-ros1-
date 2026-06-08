# ROS1 环境与雷达说明

## 关键环境变量

小车端推荐环境：

```bash
source ~/eprobot_env.sh
source ~/robot_ws/devel/setup.bash
export ROBOT_TYPE=EPRobotV2.2
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
unset ROS_HOSTNAME
```

其中 `ROBOT_TYPE=EPRobotV2.2` 是雷达能正常启动的关键配置。现场排查发现小车实际雷达应走 `ls01d` 驱动分支。

## 期望节点

底盘和定位栈正常启动后，应能看到：

```text
/base_control
/ls01d
/map_server
/talos_laser_loc
```

比赛辅助节点正常启动后，应能看到：

```text
/camera/driver
/qrcode_voice
/judge_bridge
/manual_arrival_voice
/auto_arrival_detector
```

## 关键话题

```bash
rostopic list | grep scan
rostopic echo -n 1 /scan
rostopic echo -n 1 /camera/rgb/image_raw
rostopic info /cmd_vel
```

`/cmd_vel` 在键盘控制前应没有发布者。键盘控制启动后，发布者应只有键盘节点。

## 地图

现场固定地图：

```text
maps/competition/compitation_real_3p8x4p9.yaml
maps/competition/compitation_real_3p8x4p9.pgm
```

对应场地尺寸：

```text
3.8 m x 4.9 m
resolution = 0.016 m/pixel
```

小车端实际运行时，地图应放到：

```text
/home/EPRobot/robot_ws/src/robot_navigation/maps/compitation_real_3p8x4p9.yaml
```

## 启动底盘定位

```bash
roslaunch robot_navigation robot_race_init.launch \
  map_file:=/home/EPRobot/robot_ws/src/robot_navigation/maps/compitation_real_3p8x4p9.yaml \
  open_rviz:=false
```

## 常见问题

- 如果看不到 `/ls01d`，优先检查 `ROBOT_TYPE` 是否为 `EPRobotV2.2`。
- 如果 RViz 中小车位姿跳动，先不要启用自主巡航。
- 如果键盘突然失效，先检查 SSH 是否断开，再检查 `/cmd_vel` 是否被其他节点占用。
- 如果裁判桥连接失败，检查电脑端裁判软件是否监听 `8888`，以及 Windows 防火墙是否放行该端口。
