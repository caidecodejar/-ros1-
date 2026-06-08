Write-Host "EPRobot keyboard control window"
Write-Host "Enter the robot SSH password when prompted."
Write-Host "Keys: w/a/d/s, Space stop, x exit"
Write-Host "SSH keepalive enabled: ServerAliveInterval=5 ServerAliveCountMax=6"
Write-Host ""

$remoteCommand = "bash -lc 'source ~/eprobot_env.sh; source ~/robot_ws/devel/setup.bash; export ROBOT_TYPE=EPRobotV2.2; export ROS_MASTER_URI=http://192.168.12.1:11311; export ROS_IP=192.168.12.1; unset ROS_HOSTNAME; rosnode list 2>/dev/null | grep keyboard_cmd_vel | xargs -r rosnode kill 2>/dev/null || true; rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID ""{}"" >/dev/null 2>&1 || true; rosnode kill /move_base 2>/dev/null || true; python ~/smart_pharmacy_patrol_runtime/keyboard_cmd_vel_mapping.py _speed:=0.08 _turn:=0.28 _key_timeout:=0.25'"
& ssh.exe -o ServerAliveInterval=5 -o ServerAliveCountMax=6 -o TCPKeepAlive=yes -tt EPRobot@192.168.12.1 $remoteCommand
