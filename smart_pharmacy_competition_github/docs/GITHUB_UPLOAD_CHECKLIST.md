# GitHub Upload Checklist

Before pushing this repository to GitHub:

1. Confirm secrets are absent:

   ```bash
   rg -n "<REAL_PASSWORD>|password|EPROBOT_PASSWORD=.*" .
   ```

   `EPROBOT_PASSWORD` may appear in documentation as an environment-variable name, but the real password must not appear.

2. Confirm excluded binaries are absent:

   ```bash
   rg --files | rg "\\.(exe|msi|deb|zip|tar\\.gz|tgz|rar|7z)$"
   ```

3. Confirm generated caches are absent:

   ```bash
   rg --files | rg "__pycache__|\\.pyc$|\\.log$"
   ```

4. Review what will be committed:

   ```bash
   git status
   git diff --cached --stat
   ```

5. Recommended first commit:

   ```bash
   git init
   git add .
   git commit -m "Initial smart pharmacy ROS1 competition code"
   ```

## Notes

- Do not upload the judge software executable.
- Do not upload the real robot SSH password.
- Do not upload runtime logs or temporary screenshots unless they are deliberately selected as report material.
- Keep the map and waypoint files under `maps/competition/` and the ROS runtime package under `ros1_smart_pharmacy_patrol/`.
