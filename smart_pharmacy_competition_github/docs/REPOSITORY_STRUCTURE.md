# Repository Structure

```text
smart_pharmacy_competition_github/
├── README.md
├── requirements-pc.txt
├── ros1_smart_pharmacy_patrol/
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   └── sounds/qr_voice/
├── pc_tools/
│   ├── keyboard_control_auto_login.py
│   ├── start_keyboard_control_auto_login.ps1
│   ├── start_keyboard_control.ps1
│   ├── start_keyboard_control_default.ps1
│   ├── start_judge_deploy_after_charge.ps1
│   └── web_rviz_server.py
├── tools/
│   ├── deploy_judge_system_to_robot.py
│   ├── regenerate_competition_routes.py
│   └── build_competition_submission.py
├── maps/competition/
│   ├── compitation_real_3p8x4p9.yaml
│   ├── compitation_real_3p8x4p9.pgm
│   ├── final_rviz_map_20260603.yaml
│   ├── final_rviz_map_20260603.pgm
│   ├── waypoints_real.yaml
│   ├── competition_navigation_route.yaml
│   ├── competition_navigation_planned_paths.yaml
│   └── judge/route check files
└── docs/
    ├── CODE_ROLE_AND_RULE_CHECK.md
    ├── QR_VOICE_OPERATION.md
    ├── ROS1_ENV_AND_LIDAR_NOTES.md
    ├── GITHUB_UPLOAD_CHECKLIST.md
    ├── judge_software_connection_guide.md
    ├── manual_competition_mode_20260605.md
    └── other competition notes
```

## Upload Policy

Keep source code, maps, waypoint YAML/CSV, and documentation.

Do not upload:

- real robot SSH password;
- judge software executable;
- temporary screenshots;
- runtime logs;
- generated deployment archives;
- Python cache directories.
