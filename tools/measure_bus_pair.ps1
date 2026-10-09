# =============================================================================
#  measure_bus_pair.ps1 — 同时抓 ESP32 + F411，做**增量对账**
# =============================================================================
#  为什么需要"同时 + 增量"：
#    两个板子的计数器是**各自累计**的，运行时长不同（实测从机跑过 8.6 小时、
#    主机可能刚重启），所以直接比绝对值会得出荒谬结论
#    （比如"从机收到 29245 个请求，主机只发了 3008 个"）。
#    本脚本在**同一时间窗口**内取两次读数，用**增量**对比，才可比。
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\measure_bus_pair.ps1
#    powershell -ExecutionPolicy Bypass -File tools\measure_bus_pair.ps1 -Seconds 120
#
#  输出：一张对账表 + 自动判定丢包发生在哪一段
# =============================================================================

param(
    [string] $MasterPort = 'COM6',   # ESP32 主站
    [string] $SlavePort  = 'COM8',   # F411 从站
    [int]    $Baud       = 115200,
    [int]    $Seconds    = 90
)

$ErrorActionPreference = 'Stop'

function Open-Port($name, $baud) {
    $sp = New-Object System.IO.Ports.SerialPort $name, $baud, 'None', 8, 'One'
    $sp.ReadTimeout = 50
    $sp.DtrEnable = $false
    $sp.RtsEnable = $false
    try { $sp.Open() } catch {
        Write-Host "  ✗ 打不开 $name : $($_.Exception.Message)" -ForegroundColor Red
        return $null
    }
    Start-Sleep -Milliseconds 200
    try { $sp.DiscardInBuffer() } catch { }
    return $sp
}

function Pump($port, $sb, $ms) {
    $buf = New-Object byte[] 8192
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalMilliseconds -lt $ms) {
        try {
            $n = $port.Read($buf, 0, $buf.Length)
            if ($n -gt 0) { [void]$sb.Append([System.Text.Encoding]::UTF8.GetString($buf, 0, $n)) }
        } catch { }
    }
}

# ---- 主站心跳：TX / RX / 超时 / 重试 ----
function ParseMaster($text) {
    $m = [regex]::Matches($text, 'TX\s+(\d+)\s+RX\s+(\d+)\s+超时\s+(\d+)\s+重试\s+(\d+)\s+异常\s+(\d+)\s+CRC错\s+(\d+)')
    if ($m.Count -eq 0) { return $null }
    $l = $m[$m.Count - 1]
    return [pscustomobject]@{
        TX = [int]$l.Groups[1].Value; RX = [int]$l.Groups[2].Value
        TO = [int]$l.Groups[3].Value; RT = [int]$l.Groups[4].Value
        EX = [int]$l.Groups[5].Value; CR = [int]$l.Groups[6].Value
    }
}

# ---- 从站 [mb] 行：rx / tx / parse0 / 字节数 ----
function ParseSlave($text) {
    $m = [regex]::Matches($text,
        '\[mb\]\s+rx=(\d+)\([^)]*?(\d+)B\)\s+parse0=(\d+)\s+tx=(\d+)\((\d+)B\)')
    if ($m.Count -eq 0) { return $null }
    $l = $m[$m.Count - 1]
    return [pscustomobject]@{
        RxEv = [int]$l.Groups[1].Value; RxB = [int]$l.Groups[2].Value
        P0   = [int]$l.Groups[3].Value
        TxFr = [int]$l.Groups[4].Value; TxB = [int]$l.Groups[5].Value
    }
}

Write-Host ""
Write-Host "同时抓 $MasterPort(主站) + $SlavePort(从站)，共 $Seconds 秒" -ForegroundColor Cyan
Write-Host ""

$pm = Open-Port $MasterPort $Baud
$ps = Open-Port $SlavePort  $Baud
if ($null -eq $pm -or $null -eq $ps) { exit 1 }

# ---- 第 1 次读数（窗口起点）----
$sb1 = New-Object System.Text.StringBuilder
$sb2 = New-Object System.Text.StringBuilder

Pump $pm $sb1 7000
Pump $ps $sb2 3000
$m0 = ParseMaster $sb1.ToString()
$s0 = ParseSlave  $sb2.ToString()
if ($null -eq $m0 -or $null -eq $s0) {
    Write-Host "✗ 起始读数没抓到：主站=$($null -ne $m0) 从站=$($null -ne $s0)" -ForegroundColor Red
    $pm.Close(); $ps.Close(); exit 1
}
Write-Host "  起始读数已取得，开始正式采样..." -ForegroundColor DarkGray

# ---- 正式采样窗口 ----
$sb1 = New-Object System.Text.StringBuilder
$sb2 = New-Object System.Text.StringBuilder
$sw  = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    Pump $pm $sb1 1500
    Pump $ps $sb2 1500
}
$m1 = ParseMaster $sb1.ToString()
$s1 = ParseSlave  $sb2.ToString()
$pm.Close(); $ps.Close()

if ($null -eq $m1 -or $null -eq $s1) {
    Write-Host "✗ 结束读数没抓到" -ForegroundColor Red; exit 1
}

# ---- 增量 ----
$dTx   = $m1.TX - $m0.TX      # 主站发送尝试
$dRx   = $m1.RX - $m0.RX      # 主站收到响应
$dTO   = $m1.TO - $m0.TO
$dRT   = $m1.RT - $m0.RT
$dSrxE = $s1.RxEv - $s0.RxEv  # 从站收到的事件
$dSrxB = $s1.RxB  - $s0.RxB
$dStxB = $s1.TxB  - $s0.TxB
$dP0   = $s1.P0   - $s0.P0

$rate = if ($dTx -gt 0) { 100.0 * $dRx / $dTx } else { 0 }

Write-Host ""
Write-Host "──────────── 增量对账（$Seconds 秒窗口）────────────" -ForegroundColor Green
Write-Host ("  主站发送尝试 : {0}" -f $dTx)
Write-Host ("  主站收到响应 : {0}   首试成功率 {1:N1}%" -f $dRx, $rate) -ForegroundColor Yellow
Write-Host ("  超时/重试    : {0} / {1}" -f $dTO, $dRT)
Write-Host ("  从站收到请求 : {0} 个事件（{1} 字节）" -f $dSrxE, $dSrxB)
Write-Host ("  从站发出响应 : {0} 字节" -f $dStxB)
Write-Host ("  从站静默丢弃 : {0}" -f $dP0)
Write-Host ""

# ---- 自动判定 ----
Write-Host "──────────── 判定 ────────────" -ForegroundColor Green
$verdict = @()

if ($dSrxE -lt ($dTx * 0.8)) {
    $verdict += "从站只收到 $dSrxE 个请求（主站发了 $dTx 次）→ **主站发送侧丢**：DE 时序或发送被截断"
} else {
    $verdict += "从站收到了主站的请求（$dSrxE vs $dTx）→ 主站发送**正常** ✓"
}

if ($dStxB -gt 0 -and $dRx -lt ($dTx * 0.8)) {
    $verdict += "从站发出了 $dStxB 字节，主站只收到 $dRx 个响应 → **响应回程丢**：主站接收侧（DE 切回时机 / RX 残留）"
}

if ($dSrxE -gt ($dTx * 1.5)) {
    $verdict += "从站收到的事件数（$dSrxE）远多于主站发送数（$dTx）→ **从站在自己收自己的回声**，检查模块 DIR 接线"
}

if ($dP0 -gt ($dSrxE * 0.2)) {
    $verdict += "从站静默丢弃了 $dP0 帧 → 协议层（地址/CRC）问题"
}

if ($dRx -gt 0 -and $dRx -ge ($dTx * 0.95)) {
    $verdict = @("✓ 通信良好，无需处理")
}

foreach ($v in $verdict) { Write-Host ("  · " + $v) }
Write-Host ""
