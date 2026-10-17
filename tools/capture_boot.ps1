# =============================================================================
#  capture_boot.ps1 — 自动抓取串口启动日志（不用盯着屏幕，不漏内容）
# =============================================================================
#  为什么需要这个：
#    排查"板子卡在哪一步"时，人工盯着串口窗口容易漏掉（启动打印一闪而过，
#    而且窗口不会自动保存）。这个脚本先开串口，再等你复位板子，
#    把收到的一切都写进文件，最后自动分析"最后一条打印是什么"。
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\capture_boot.ps1 -Port COM8 -Sec 12
#
#  流程：
#    1) 脚本打开串口
#    2) 提示你"现在按板子复位键"（有 3 秒准备时间）
#    3) 抓 12 秒（-Sec 可调），期间所有串口数据存到 logs\boot_capture.txt
#    4) 结束后自动打印：收到多少字节、最后一段内容、最后一条 [BOOT] 行
# =============================================================================

param(
    [string] $Port,
    [int]    $Baud = 115200,
    [int]    $Sec  = 12,
    [string] $Out  = "logs\boot_capture.txt"
)

$ErrorActionPreference = 'Stop'

# ---- 选串口 ---------------------------------------------------------------
if (-not $Port) {
    $names = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if (-not $names) { Write-Host "没有可用串口" -ForegroundColor Red; exit 1 }
    if ($names.Count -eq 1) { $Port = $names[0] }
    else {
        Write-Host "可用串口：" -ForegroundColor Cyan
        $names | ForEach-Object { Write-Host "  $_" }
        $Port = Read-Host "输入串口名"
    }
}

# ---- 准备输出目录 ---------------------------------------------------------
$outPath = Join-Path (Get-Location) $Out
$outDir  = Split-Path $outPath -Parent
if (-not (Test-Path $outDir)) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }

# ---- 打开串口 -------------------------------------------------------------
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 100
$sp.DtrEnable   = $false      # 不拉 DTR，避免某些板子被这个信号复位
$sp.RtsEnable   = $false
try { $sp.Open() }
catch {
    Write-Host ("打不开 {0}：{1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host "串口可能被串口助手/其它窗口占着，先全部关掉。" -ForegroundColor Yellow
    exit 1
}
Write-Host ""
Write-Host ("已打开 {0} @ {1}" -f $Port, $Baud) -ForegroundColor Green
Write-Host "=== 现在去按板子上的复位键（或拔插 Type-C 重新上电）===" -ForegroundColor Yellow
Write-Host ("准备 3 秒...") -ForegroundColor DarkGray
Start-Sleep -Seconds 3
Write-Host ("开始抓取 {0} 秒 ..." -f $Sec) -ForegroundColor Cyan

# ---- 抓取 -----------------------------------------------------------------
$sb  = New-Object System.Text.StringBuilder
$buf = New-Object byte[] 4096
$t0  = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt $Sec) {
    try {
        $n = $sp.Read($buf, 0, $buf.Length)
        if ($n -gt 0) {
            [void]$sb.Append([System.Text.Encoding]::UTF8.GetString($buf, 0, $n))
        }
    } catch [TimeoutException] { }
      catch { }
}
$sp.Close()

# ---- 保存 -----------------------------------------------------------------
$text = $sb.ToString()
[System.IO.File]::WriteAllText($outPath, $text, [System.Text.UTF8Encoding]::new($false))

# ---- 分析 -----------------------------------------------------------------
Write-Host ""
Write-Host ("=" * 62) -ForegroundColor Cyan
Write-Host ("收到 {0} 字节" -f $text.Length) -ForegroundColor Green
Write-Host ("已保存到：{0}" -f $outPath) -ForegroundColor Green
Write-Host ("=" * 62) -ForegroundColor Cyan

if ($text.Length -eq 0) {
    Write-Host "⚠ 一个字节都没收到！可能原因：" -ForegroundColor Red
    Write-Host "  1) TX/RX 接反（板子的 PA9 要接 USB-TTL 的 RXD）" -ForegroundColor Yellow
    Write-Host "  2) 板子没在运行（复位后程序卡住 / 没烧写成功）" -ForegroundColor Yellow
    Write-Host "  3) GND 没接（必须有公共地）" -ForegroundColor Yellow
    Write-Host "  4) 波特率不对（本工程固定 115200）" -ForegroundColor Yellow
    exit 2
}

Write-Host ""
Write-Host "---- 收到的内容（原样）----" -ForegroundColor Cyan
Write-Host $text

Write-Host ""
Write-Host "---- 自动分析 ----" -ForegroundColor Cyan
$lines = $text -split "`r?`n" | Where-Object { $_.Trim() -ne '' }
$boot  = $lines | Where-Object { $_ -match '\[BOOT\]' }
if ($boot) {
    Write-Host "启动进度（[BOOT] 行）：" -ForegroundColor Green
    $boot | ForEach-Object { Write-Host ("  {0}" -f $_.Trim()) }
    Write-Host ""
    Write-Host ("★ 最后到达的一步： {0}" -f $boot[-1].Trim()) -ForegroundColor Yellow
} else {
    Write-Host "没有 [BOOT] 行 —— 可能烧的还是旧固件，或板子卡在 BOOT 之前。" -ForegroundColor Yellow
}
$audio = $lines | Where-Object { $_ -match '\[AUDIO\]' }
if ($audio) {
    Write-Host ""
    Write-Host "[AUDIO] 行：" -ForegroundColor Green
    $audio | ForEach-Object { Write-Host ("  {0}" -f $_.Trim()) }
}
