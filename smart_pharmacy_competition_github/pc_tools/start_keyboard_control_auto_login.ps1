Write-Host "EPRobot keyboard control - auto login, speed 0.25 m/s"
Write-Host "Set EPROBOT_PASSWORD before running to skip the password prompt."
Write-Host ""

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$logDir = Join-Path $repoRoot "logs"
$logFile = Join-Path $logDir "keyboard_control_auto_login.log"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
Start-Transcript -Path $logFile -Force | Out-Null

try {
    python -u (Join-Path $PSScriptRoot "keyboard_control_auto_login.py")
    Write-Host ""
    Write-Host "Keyboard launcher exited with code $LASTEXITCODE"
}
finally {
    Stop-Transcript | Out-Null
    Write-Host "Log: $logFile"
}
