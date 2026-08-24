# fix_d435.ps1 —— 一键把 D435 恢复到 WSL（修复“设备无效/看不到摄像头”）
# 用法：在【管理员身份运行】的 PowerShell 中执行：
#   cd \\wsl.localhost\Ubuntu-22.04-F\home\zhj\projects\fr5_platform_ws\scripts
#   powershell -ExecutionPolicy Bypass -File .\fix_d435.ps1
$ErrorActionPreference = "Continue"
$OutputEncoding = [System.Text.Encoding]::UTF8
$wslDistro = "Ubuntu-22.04-F"
$proj = "/home/zhj/projects/fr5_platform_ws"

# 0) 管理员检查
$id = [Security.Principal.WindowsIdentity]::GetCurrent()
$isAdmin = (New-Object Security.Principal.WindowsPrincipal($id)).IsInRole("Administrator")
if (-not $isAdmin) {
    Write-Host "ERROR: 请以【管理员身份运行】PowerShell 后再执行本脚本。" -ForegroundColor Red
    exit 1
}

function Find-D435BusId {
    $out = usbipd list 2>&1 | Out-String
    foreach ($line in ($out -split "`n")) {
        if ($line -match "8086:0b07") {
            return ($line.Trim() -split "\s+")[0]
        }
    }
    return $null
}

Write-Host "[1/5] 强制重枚举 USB 设备（让 Windows 重新扫描所有口）..." -ForegroundColor Cyan
pnputil /scan-devices 2>&1 | Out-Null
Start-Sleep -Seconds 5

$busid = Find-D435BusId
if (-not $busid) {
    Write-Host "ERROR: usbipd 中仍未发现 8086:0b07 (D435)。" -ForegroundColor Red
    Write-Host "请确认：D435 已【直接】插在主板背面的 USB 3.0 口（蓝色 SS，不要经过任何集线器/扩展坞），" -ForegroundColor Yellow
    Write-Host "使用原装 USB 3.0 线（蓝色头），拔下后等 5 秒再插，然后重新运行本脚本。" -ForegroundColor Yellow
    exit 1
}
Write-Host "[2/5] 发现 D435，busid = $busid" -ForegroundColor Green

Write-Host "[3/5] 绑定 (bind) 以安装 WinUSB 驱动..." -ForegroundColor Cyan
usbipd bind --busid $busid 2>&1 | Out-String

Write-Host "[4/5] 附加到 WSL ($wslDistro)..." -ForegroundColor Cyan
usbipd attach --wsl $wslDistro --busid $busid 2>&1 | Out-String
Start-Sleep -Seconds 3

Write-Host "[5/5] 验证附加状态：" -ForegroundColor Cyan
usbipd list 2>&1 | Select-String "8086:0b07" | Out-String

Write-Host "重启 fr5 平台服务（相机/状态/界面）..." -ForegroundColor Cyan
wsl -d $wslDistro -- bash -c "cd $proj && bash scripts/platforms.sh restart" 2>&1 | Out-String
Start-Sleep -Seconds 15

Write-Host "验证 WSL 是否看到 D435 与视频设备：" -ForegroundColor Cyan
wsl -d $wslDistro -- bash -c "lsusb | grep 8086:0b07; echo '---'; ls /dev/video* 2>/dev/null || echo NO_VIDEO" 2>&1 | Out-String

Write-Host "完成。请刷新 http://127.0.0.1:8080/ 查看设备状态；若仍无效，多半是 USB 2.0 协商（线/转接问题），需换 USB 3.0 线。" -ForegroundColor Green
