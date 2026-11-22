#requires -RunAsAdministrator
<#
.SYNOPSIS
    WSL2 + Ubuntu 22.04 + ROS2 Humble 一键安装（重启后运行）

.DESCRIPTION
    背景：本机此前没有任何 WSL 发行版，Windows 功能
    Microsoft-Windows-Subsystem-Linux 与 VirtualMachinePlatform 已启用，
    但**必须重启一次**才会生效。

    重启后以管理员身份运行本脚本，它会：
      1. 检查 WSL / 虚拟化功能状态；
      2. 确保 WSL2 运行时可用（必要时 wsl --install --no-distribution / --update）；
      3. 用清华镜像的官方 rootfs 把 Ubuntu 22.04 导入到 D:\wsl\k1-ubuntu（不占 C 盘）；
      4. 在 Ubuntu 内换源、装基础工具、装 ROS2 Humble Desktop；
      5. 创建用户 k1 并设为默认用户；
      6. 跑一遍验证。

.NOTES
    强烈建议直接用 setup_wsl_after_reboot.cmd 启动（会自动申请管理员并记日志）。
#>

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$RootDir    = 'D:\vision_ws'
$SetupDir   = Join-Path $RootDir '_setup'
$Rootfs     = Join-Path $SetupDir 'ubuntu-jammy-wsl-amd64-rootfs.tar.gz'
$Distro     = 'k1-ubuntu'
$InstallDir = 'D:\wsl\k1-ubuntu'
$LogFile    = Join-Path $SetupDir 'wsl_setup.log'
$InsideSh   = 'D:\vision_ws\k1_soccer_vision\scripts\setup_inside_wsl.sh'

function Log([string]$msg) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $msg
    Write-Host $line
    Add-Content -Path $LogFile -Value $line -Encoding UTF8
}

function Test-Admin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    $p = New-Object Security.Principal.WindowsPrincipal($id)
    return $p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

if (-not (Test-Admin)) {
    Write-Host "需要管理员权限：请改用 setup_wsl_after_reboot.cmd 运行。" -ForegroundColor Red
    exit 1
}

New-Item -ItemType Directory -Force -Path $SetupDir | Out-Null
Add-Content -Path $LogFile -Value "`n`n========== $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ==========" -Encoding UTF8

Log "=== 步骤 1/6：检查 Windows 功能 ==="
$features = Get-WindowsOptionalFeature -Online |
    Where-Object { $_.FeatureName -in @('Microsoft-Windows-Subsystem-Linux', 'VirtualMachinePlatform') }
$features | ForEach-Object { Log ("  {0} = {1}" -f $_.FeatureName, $_.State) }
if ($features | Where-Object { $_.State -ne 'Enabled' }) {
    Log "功能未全部启用，正在启用（完成后需要重启）..."
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart | Out-Null
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart | Out-Null
    Log "已启用，请重启后再次运行本脚本。"
    exit 2
}

Log "=== 步骤 2/6：检查 WSL2 运行时 ==="
$wslOk = $false
try {
    $null = & wsl.exe --status 2>&1
    if ($LASTEXITCODE -eq 0) { $wslOk = $true }
} catch { }

if (-not $wslOk) {
    Log "WSL 运行时不可用，尝试安装（从 Microsoft CDN 下载 WSL 本体）..."
    & wsl.exe --install --no-distribution 2>&1 | ForEach-Object { Log "  $_" }
    Start-Sleep -Seconds 5
    try { $null = & wsl.exe --status 2>&1; if ($LASTEXITCODE -eq 0) { $wslOk = $true } } catch { }
}

if (-not $wslOk) {
    Log "仍然不可用，尝试 wsl --update ..."
    & wsl.exe --update 2>&1 | ForEach-Object { Log "  $_" }
    Start-Sleep -Seconds 5
    try { $null = & wsl.exe --status 2>&1; if ($LASTEXITCODE -eq 0) { $wslOk = $true } } catch { }
}

if (-not $wslOk) {
    Log "!! WSL 运行时仍不可用。可能是重启尚未完成，或需要手动执行 'wsl --install --no-distribution'。"
    Log "   本脚本到此停止，其余步骤可在 WSL 可用后重新运行。"
    exit 3
}
Log "WSL 运行时可用。"

& wsl.exe --set-default-version 2 2>&1 | ForEach-Object { Log "  $_" }

Log "=== 步骤 3/6：导入 Ubuntu 22.04 到 $InstallDir ==="
if (-not (Test-Path $Rootfs)) {
    Log "!! 找不到 rootfs：$Rootfs"
    Log "   请重新下载："
    Log "   curl.exe -L -o `"$Rootfs`" https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cloud-images/wsl/jammy/current/ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz"
    exit 4
}

$existing = (& wsl.exe -l -q 2>&1) -replace "`0", ''
if ($existing -match [regex]::Escape($Distro)) {
    Log "发行版 $Distro 已存在，跳过导入。"
} else {
    New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
    Log "导入中（325MB rootfs，约 1~3 分钟）..."
    $importArgs = @('--import', $Distro, $InstallDir, $Rootfs, '--version', '2')
    & wsl.exe @importArgs 2>&1 | ForEach-Object { Log "  $_" }
    if ($LASTEXITCODE -ne 0) {
        Log "!! 导入失败，退出码 $LASTEXITCODE"
        exit 5
    }
    Log "导入完成。"
}

Log "=== 步骤 4/6：写 /etc/wsl.conf ==="
$wslConf = @"
[boot]
systemd=true

[interop]
enabled=true
appendWindowsPath=false

[automount]
enabled=true
options="metadata,umask=22,fmask=11"

[user]
default=k1
"@
# 用 LF 写入，避免 WSL 里出现 CRLF
$wslConf | ForEach-Object { $_ } | Out-Null
$tmp = Join-Path $SetupDir 'wsl.conf.tmp'
[System.IO.File]::WriteAllText($tmp, ($wslConf -replace "`r`n", "`n"), (New-Object System.Text.UTF8Encoding($false)))
& wsl.exe -d $Distro -u root -- bash -c "mkdir -p /etc && cat > /etc/wsl.conf < '/mnt/d/vision_ws/_setup/wsl.conf.tmp'" 2>&1 | ForEach-Object { Log "  $_" }
& wsl.exe -d $Distro -u root -- cat /etc/wsl.conf 2>&1 | ForEach-Object { Log "  $_" }

Log "=== 步骤 5/6：Ubuntu 内安装 ROS2 Humble（最慢，约 10~25 分钟）==="
& wsl.exe -d $Distro -u root -- bash /mnt/d/vision_ws/k1_soccer_vision/scripts/setup_inside_wsl.sh 2>&1 |
    ForEach-Object { Log "  $_" }
if ($LASTEXITCODE -ne 0) {
    Log "!! Ubuntu 内部安装失败，退出码 $LASTEXITCODE，详见上方日志。"
    Log "   可单独重跑：wsl -d $Distro -u root -- bash /mnt/d/vision_ws/k1_soccer_vision/scripts/setup_inside_wsl.sh"
    exit 6
}

Log "=== 步骤 6/6：创建用户 k1（默认用户）==="
$userScript = @'
id -u k1 >/dev/null 2>&1 || useradd -m -s /bin/bash k1
echo 'k1:k1' | chpasswd
usermod -aG sudo,adm,video,dialout,plugdev k1
echo 'k1 ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-k1
chmod 440 /etc/sudoers.d/90-k1
id k1
'@
$tmpUser = Join-Path $SetupDir 'makeuser.sh'
[System.IO.File]::WriteAllText($tmpUser, ($userScript -replace "`r`n", "`n"), (New-Object System.Text.UTF8Encoding($false)))
& wsl.exe -d $Distro -u root -- bash /mnt/d/vision_ws/_setup/makeuser.sh 2>&1 | ForEach-Object { Log "  $_" }

Log "=== 重启 WSL 使配置生效 ==="
& wsl.exe --terminate $Distro 2>&1 | Out-Null
Start-Sleep -Seconds 3

Log "=== 验证 ==="
& wsl.exe -d $Distro -u k1 -- bash -lc "echo user=`$(whoami); echo ubuntu=`$(. /etc/os-release; echo `$PRETTY_NAME); echo ROS_DISTRO=`$ROS_DISTRO; echo ROS_VERSION=`$ROS_VERSION; ros2 pkg list 2>/dev/null | wc -l; echo kb=`$(uname -r)" 2>&1 |
    ForEach-Object { Log "  $_" }

Log "全部完成。命令： wsl -d $Distro"
Log "日志： $LogFile"
