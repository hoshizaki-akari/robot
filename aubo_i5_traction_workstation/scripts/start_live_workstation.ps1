param(
    [string]$Distro = 'Ubuntu-22.04-F',
    [string]$ProjectPath = '/home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation',
    [Parameter(Mandatory = $true)][string]$CalibrationFile,
    [switch]$Background
)

$ErrorActionPreference = 'Stop'
$shellExecutable = (Get-Process -Id $PID).Path
# Supply the already established SDK credentials through this process environment.
# They are inherited, never written to command lines, logs, or the repository.
foreach ($name in @('AUBO_V4_USERNAME', 'AUBO_V4_PASSWORD')) {
    if (-not [Environment]::GetEnvironmentVariable($name)) {
        throw "Missing $name in this process environment. No hardware was started."
    }
}
foreach ($path in @($ProjectPath, $CalibrationFile)) {
    if ($path -notmatch '^/[A-Za-z0-9_./-]+$') {
        throw 'Use an absolute WSL path without spaces or shell metacharacters.'
    }
}
$localProject = Split-Path $PSScriptRoot -Parent
$runName = 'session_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '_workstation'
$logDirectory = Join-Path $localProject ('debug/' + $runName)
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
if ($Background) {
    $arguments = @('-NoProfile', '-File', ('"' + $PSCommandPath + '"'),
        '-Distro', ('"' + $Distro + '"'), '-ProjectPath', $ProjectPath,
        '-CalibrationFile', $CalibrationFile)
    $process = Start-Process $shellExecutable -ArgumentList $arguments -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $logDirectory 'supervisor_windows.log') `
        -RedirectStandardError (Join-Path $logDirectory 'supervisor_windows_error.log')
    Write-Output "Background supervisor PID=$($process.Id); launcher logs=$logDirectory"
    exit 0
}
foreach ($port in @(8082, 18755)) {
    if (Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue) {
        throw "Port $port is already owned. Stop the previous workstation first; no duplicate was started."
    }
}
$forwardNames = @('AUBO_V4_HOST', 'AUBO_V4_PORT', 'AUBO_V4_USERNAME', 'AUBO_V4_PASSWORD',
    'AUBO_V4_SDK_ROOT', 'AUBO_V4_PROTOBUF_DIR')
$existingForward = @($env:WSLENV -split ':' | Where-Object { $_ })
$env:WSLENV = (@($existingForward + $forwardNames) | Select-Object -Unique) -join ':'
$bridge = $null
try {
    $bridge = Start-Process $shellExecutable -ArgumentList @('-NoProfile', '-File',
        ('"' + (Join-Path $PSScriptRoot 'kwr75_windows_bridge.ps1') + '"')) `
        -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $logDirectory 'windows_sensor_bridge.log') `
        -RedirectStandardError (Join-Path $logDirectory 'windows_sensor_bridge_error.log')
    $listening = $false
    for ($attempt = 0; $attempt -lt 50; $attempt++) {
        $bridge.Refresh()
        if ($bridge.HasExited) { throw 'COM4 bridge exited; inspect its error log.' }
        if (Get-NetTCPConnection -LocalPort 18755 -State Listen -ErrorAction SilentlyContinue) {
            $listening = $true
            break
        }
        Start-Sleep -Milliseconds 100
    }
    if (-not $listening) { throw 'COM4 bridge did not become ready.' }
    $wslLogs = "$ProjectPath/debug/$runName"
    & wsl.exe -d $Distro --exec /bin/bash "$ProjectPath/scripts/run_live_workstation.sh" `
        $CalibrationFile $wslLogs
    if ($LASTEXITCODE -ne 0) { throw "WSL supervisor exited with code $LASTEXITCODE. Logs: $wslLogs" }
}
finally {
    if ($bridge) {
        $bridge.Refresh()
        if (-not $bridge.HasExited) { Stop-Process -Id $bridge.Id }
    }
}
