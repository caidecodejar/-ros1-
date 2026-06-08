Write-Host "Deploy judge-system-ready package to EPRobot"
Write-Host "Use after the robot is charged and its WiFi is connected."
Write-Host "Set EPROBOT_PASSWORD before running to skip the password prompt."
Write-Host ""

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$deployTool = Join-Path $repoRoot "tools\deploy_judge_system_to_robot.py"
$package = Join-Path $repoRoot "dist\judge_system_match_ready_20260605_clean.tar.gz"

if (-not (Test-Path -LiteralPath $package)) {
    Write-Host "Package not found: $package"
    Write-Host "Build or place the deployment tarball under dist before running this helper."
    exit 1
}

python -u $deployTool `
    --host 192.168.12.1 `
    --user EPRobot `
    --source-ip 192.168.12.248 `
    --package $package
