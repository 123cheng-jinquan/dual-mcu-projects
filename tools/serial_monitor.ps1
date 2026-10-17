# =============================================================================
#  serial_monitor.ps1 — 串口监视 + 发送（零依赖，用 .NET SerialPort）
# =============================================================================
#  为什么要自己写这个：
#    项目里没有串口助手，而验证采样必须能"发命令 + 看回复"（比如敲 t 触发
#    音频自检）。Keil 自带的 UART 窗口只能看不能发，VS Code 也不是串口工具。
#    这个脚本零安装、双击就能用。
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
#    powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1 -Port COM7 -Baud 115200
#    powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1 -Port COM7 -Send "t"
#
#  操作：
#    · 直接敲字符再按回车 → 发出去（会显示 [TX] 方便核对）
#    · 输入 :quit 或按 Ctrl+C → 退出
#    · 收到的数据原样显示（含中文，按 UTF-8 解码；本项目已改成纯 ASCII）
#
#  ⚠ 如果列出多个串口而你不确定是哪个：看"设备名"那一列，
#    CH340/CH343/CP2102/FTDI 这些才是 USB-TTL；DAP-Link 不会出现在串口列表里。
# =============================================================================

param(
    [string] $Port,
    [int]    $Baud = 115200,
    [string] $Send,                 # 打开后自动发一条命令（如 -Send "t"）
    [int]    $AutoSendEverySec = 0, # >0 时每隔 N 秒自动重发 -Send
    [string] $Log                   # 可选：把收到的内容同时写进文件
)

$ErrorActionPreference = 'Stop'

# ---- 列出可用串口（带设备名，方便确认接的是哪个）---------------------------
function Show-Ports {
    Write-Host ""
    Write-Host "可用的串口：" -ForegroundColor Cyan
    $names = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if (-not $names) {
        Write-Host "  （一个都没有）" -ForegroundColor Yellow
        Write-Host "  检查：USB-TTL 驱动装了吗？数据线是数据线不是纯充电线吗？" -ForegroundColor Yellow
        return @()
    }
    $devs = @{}
    try {
        Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '\(COM\d+\)' } |
            ForEach-Object {
                if ($_.Name -match '\((COM\d+)\)') {
                    $devs[$Matches[1]] = ($_.Name -replace '\s*\(COM\d+\)','')
                }
            }
    } catch { }
    foreach ($n in $names) {
        $d = if ($devs.ContainsKey($n)) { $devs[$n] } else { '(未知设备)' }
        Write-Host ("  {0,-7} {1}" -f $n, $d) -ForegroundColor Green
    }
    Write-Host ""
    return $names
}

# ---- 选串口 ---------------------------------------------------------------
if (-not $Port) {
    $avail = Show-Ports
    if ($avail.Count -eq 0) { exit 1 }
    if ($avail.Count -eq 1) {
        $Port = $avail[0]
        Write-Host "只有一个串口，自动选：$Port" -ForegroundColor Cyan
    } else {
        $Port = Read-Host "请输入要用的串口名（如 COM7）"
        if (-not $Port) { exit 1 }
    }
}

# ---- 打开串口 -------------------------------------------------------------
Write-Host ""
Write-Host ("打开 {0} @ {1} 8N1 ..." -f $Port, $Baud) -ForegroundColor Cyan

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout  = 50
$sp.WriteTimeout = 1000
$sp.DtrEnable    = $false      # 不拉 DTR/RTS：避免某些板子被复位
$sp.RtsEnable    = $false
try {
    $sp.Open()
} catch {
    Write-Host ("打不开 {0}：{1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host "可能原因：串口被别的程序占着（串口助手/Keil/监视器），先关掉。" -ForegroundColor Yellow
    exit 1
}
Write-Host "已连接。直接敲字发送（回车确认），输入 :quit 退出。" -ForegroundColor Green
Write-Host ("-" * 60)
Write-Host ""

$logWriter = $null
if ($Log) {
    $logWriter = New-Object System.IO.StreamWriter($Log, $false, [System.Text.UTF8Encoding]::new($false))
    Write-Host "同时记录到：$Log" -ForegroundColor DarkGray
}

# ---- 可选：打开后就发一条 -------------------------------------------------
if ($Send) {
    Start-Sleep -Milliseconds 300
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($Send + "`r`n")
    $sp.Write($bytes, 0, $bytes.Length)
    Write-Host ("[TX] {0}" -f $Send) -ForegroundColor Magenta
}

$lastAuto = Get-Date
$buf = New-Object byte[] 4096
$out = New-Object System.Text.StringBuilder
$lineBuf = New-Object System.Text.StringBuilder

try {
    while ($true) {
        # ---- 接收 ----
        try {
            $n = $sp.Read($buf, 0, $buf.Length)
            if ($n -gt 0) {
                $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
                Write-Host $txt -NoNewline
                if ($logWriter) { $logWriter.Write($txt); $logWriter.Flush() }
                [void]$lineBuf.Append($txt)
            }
        } catch [TimeoutException] { }
          catch { }

        # ---- 发送（按键驱动，不用 Read-Host：那个是阻塞的，会卡住接收）----
        $line = ''
        $typing = $false
        while ([Console]::KeyAvailable) {
            $k = [Console]::ReadKey($true)
            if ($k.Key -eq 'Enter') {
                $typing = $true
                break
            }
            if ($k.Key -eq 'Backspace') {
                if ($line.Length -gt 0) { $line = $line.Substring(0, $line.Length - 1) }
                continue
            }
            $ch = $k.KeyChar
            if ($ch -and [int]$ch -ge 32) { $line += $ch }
        }
        if ($typing) {
            if ($line -eq ':quit') { break }
            if ($line -ne '') {
                $bytes = [System.Text.Encoding]::ASCII.GetBytes($line + "`r`n")
                $sp.Write($bytes, 0, $bytes.Length)
                Write-Host ("[TX] {0}" -f $line) -ForegroundColor Magenta
            }
        } else {
            Start-Sleep -Milliseconds 20
        }

        # ---- 定时重发（用于周期性采集，如每 5 秒敲一次 t）----
        if ($AutoSendEverySec -gt 0 -and $Send) {
            if (((Get-Date) - $lastAuto).TotalSeconds -ge $AutoSendEverySec) {
                $lastAuto = Get-Date
                $bytes = [System.Text.Encoding]::ASCII.GetBytes($Send + "`r`n")
                $sp.Write($bytes, 0, $bytes.Length)
                Write-Host ("[TX-auto] {0}" -f $Send) -ForegroundColor DarkMagenta
            }
        }
    }
} finally {
    if ($sp.IsOpen) { $sp.Close() }
    if ($logWriter) { $logWriter.Close() }
    Write-Host ""
    Write-Host "已断开。" -ForegroundColor Cyan
}
