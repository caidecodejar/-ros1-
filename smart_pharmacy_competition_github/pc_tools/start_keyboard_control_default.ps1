Write-Host "EPRobot keyboard control window - default speed"
Write-Host "Enter the robot SSH password when prompted."
Write-Host "Keys: w/a/s/d move, Space stop, x exit"
Write-Host "Adjust speed: q faster, e slower"
Write-Host "Adjust turn: z/c"
Write-Host "SSH keepalive enabled: ServerAliveInterval=5 ServerAliveCountMax=6"
Write-Host ""

$remoteCommand = "bash -lc 'source ~/eprobot_env.sh; source ~/robot_ws/devel/setup.bash; export ROBOT_TYPE=EPRobotV2.2; export ROS_MASTER_URI=http://192.168.12.1:11311; export ROS_IP=192.168.12.1; unset ROS_HOSTNAME; rosnode list 2>/dev/null | grep keyboard_cmd_vel | xargs -r rosnode kill 2>/dev/null || true; rosnode kill /move_base 2>/dev/null || true; python ~/smart_pharmacy_patrol_runtime/keyboard_cmd_vel_mapping.py'"
& ssh.exe -o ServerAliveInterval=5 -o ServerAliveCountMax=6 -o TCPKeepAlive=yes -tt EPRobot@192.168.12.1 $remoteCommand
