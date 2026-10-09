# =============================================================================
#  measure_bus_quality.ps1 — 测量 485 总线"首试成功率"
# =============================================================================
#  为什么要这个脚本：
#    加 120Ω 终端电阻前后，需要一个**可比的数字**来判断有没有效果。
#    光看"超时 42 重试 42"这种绝对值，会随测试时长变化，没法前后对比。
#    所以这里算的是**比例**：首试成功率 = 1 - 超时/发送。
#
#  用法（默认 COM6、60 秒）：
#    powershell -ExecutionPolicy Bypass -File tools\measure_bus_quality.ps1
#    powershell -ExecutionPolicy Bypass -File tools\measure_bus_quality.ps1 -Seconds 120
#    powershell -ExecutionPolicy Bypass -File tools\measure_bus_quality.ps1 -Port COM8 -Baud 9600
#
#  ⚠ 说明：
#   · 脚本**不复位**设备，直接读它正在打印的心跳（所以别和 pio device monitor 抢串口）
#   · 判定用最后一个"总线心跳"行（累计值），比例不受采样窗口影响
#   · 重试数**恒等于**超时数是正常的：每次重试都成功（说明是首发丢帧，不是从机不回）
# =============================================================================

param(
    [string] $Port    = 'COM6',
    [int]    $Baud    = 115200,
    [int]    $Seconds = 60
)

$ErrorActionPreference = 'Stop'

Write-Host ""
Write-Host "测量 $Port 的 485 总线质量，持续 $Seconds 秒（不复位设备）..." -ForegroundColor Cyan
Write-Host ""

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
# ★ ReadTimeout 必须给**短**值 ★
#   `Read()` 是**阻塞**的：超时多长，没数据时它就等多久。
#   超时设 400ms 会让采集循环每轮只跑 2.5 次，串口驱动缓冲区来不及排空 →
#   严重丢日志（实测：45 秒只抓到 1.9KB，连 5 秒一条的心跳都抓不全，
#   表现是"抓不到心跳行"）。改成 50ms 后采集频率提高 8 倍。
$sp.ReadTimeout = 50
# 关键：不要把 DTR/RTS 拉低，否则会复位 ESP32（我们想看的是它当前的运行状态）
$sp.DtrEnable = $false
$sp.RtsEnable = $false

try { $sp.Open() } catch {
    Write-Host "✗ 打不开 $Port ：$($_.Exception.Message)" -ForegroundColor Red
    Write-Host "  （是不是 pio device monitor 还占着？先关掉它）" -ForegroundColor Yellow
    exit 1
}

$sb        = New-Object System.Text.StringBuilder
$buf       = New-Object byte[] 8192
$sw        = [System.Diagnostics.Stopwatch]::StartNew()
$lastBeat  = $null

# ── 为什么先清一次输入缓冲 ─────────────────────────────────────────────
# 打开串口时，驱动缓冲区里可能积压了几秒前的旧日志。
# 不清掉的话，"这一窗口"的统计里混着旧数据，前后对比就不准了。
# ⚠ 实测踩到：不清缓冲时 40 秒只抓到 1.6KB（缓冲区慢速溢出、大量日志丢失），
#   表现是"抓不到心跳行"。
Start-Sleep -Milliseconds 300
try { $sp.DiscardInBuffer() } catch { }

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $n = $sp.Read($buf, 0, $buf.Length)
        if ($n -gt 0) {
            [void]$sb.Append([System.Text.Encoding]::UTF8.GetString($buf, 0, $n))
        }
    } catch { }
}
$sp.Close()

$text  = $sb.ToString()
$lines = $text -split "`n"

# 心跳行格式（字段间是**多个空格**，所以用 \s+）：
#   [I][gw] 总线心跳：TX 1007  RX 587  超时 420  重试 420  异常 0  CRC错 0  在线  等待处理 0
$beatRe = 'TX\s+(\d+)\s+RX\s+(\d+)\s+超时\s+(\d+)\s+重试\s+(\d+)\s+异常\s+(\d+)\s+CRC错\s+(\d+)'
$beats  = @()
foreach ($ln in $lines) {
    $m = [regex]::Match($ln, $beatRe)
    if ($m.Success) {
        $beats += [pscustomobject]@{
            TX    = [int]$m.Groups[1].Value
            RX    = [int]$m.Groups[2].Value
            Timeout = [int]$m.Groups[3].Value
            Retry = [int]$m.Groups[4].Value
            Exc   = [int]$m.Groups[5].Value
            Crc   = [int]$m.Groups[6].Value
        }
    }
}

if ($beats.Count -eq 0) {
    Write-Host "✗ 没抓到任何'总线心跳'行。" -ForegroundColor Red
    Write-Host "  可能原因：设备没在跑 / 串口选错 / 波特率不对 / 设备还在复位循环" -ForegroundColor Yellow
    Write-Host "  抓到的原始字节数：$($text.Length)" -ForegroundColor DarkGray
    exit 1
}

$last = $beats[-1]

# 首试成功率 = 成功的第一发 / 总发送
# 注意：一次"请求"可能因为超时重试多次，所以用 TX 作分母、
#      "超时次数"作失败数（每次超时 = 首发失败一次）
$success = $last.TX - $last.Timeout
if ($last.TX -le 0) { $success = 0 }
$rate = if ($last.TX -gt 0) { 100.0 * $success / $last.TX } else { 0 }

Write-Host "──────────── 结果 ────────────" -ForegroundColor Green
Write-Host ("  采样到心跳行 : {0} 条（{1} 秒）" -f $beats.Count, $Seconds)
Write-Host ("  累计 TX      : {0}" -f $last.TX)
Write-Host ("  累计 RX      : {0}" -f $last.RX)
Write-Host ("  超时         : {0}" -f $last.Timeout)
Write-Host ("  重试         : {0}" -f $last.Retry)
Write-Host ("  异常码       : {0}" -f $last.Exc)
Write-Host ("  CRC 错       : {0}" -f $last.Crc)
Write-Host ""
Write-Host ("  ★ 首试成功率 : {0:N1}%" -f $rate) -ForegroundColor Yellow
Write-Host ""

# 给出可操作的结论
if ($rate -ge 98) {
    Write-Host "  ✓ 非常好。可以切回 115200、把快车道周期改回 2 秒。" -ForegroundColor Green
    Write-Host "    （见 README §9.2 的 6 处耦合改法）" -ForegroundColor DarkGray
} elseif ($rate -ge 90) {
    Write-Host "  ✓ 良好。偶尔重试可以接受，波形页已经能正常跑。" -ForegroundColor Green
} elseif ($rate -ge 75) {
    Write-Host "  ⚠ 一般。建议在**总线另一端**再并一个 120Ω 试试。" -ForegroundColor Yellow
} else {
    Write-Host "  ✗ 偏差。检查：" -ForegroundColor Red
    Write-Host "     1. 120Ω 是否真的并在了 A-B 之间（不是 A-GND 或 B-GND）" -ForegroundColor Yellow
    Write-Host "     2. A/B 是否是双绞（两根缠在一起），有没有和电源线平行" -ForegroundColor Yellow
    Write-Host "     3. 两端 GND 是否互联" -ForegroundColor Yellow
    Write-Host "     4. 波特率是否还是 9600（改 115200 会更差）" -ForegroundColor Yellow
}

# CRC 错 / 异常单独提醒（和"丢帧"是不同性质的问题）
if ($last.Crc -gt 0) {
    Write-Host ""
    Write-Host "  ⚠ 有 CRC 错（$($last.Crc) 次）：帧间静默可能不够，或线缆受干扰" -ForegroundColor Yellow
}
if ($last.Exc -gt 0) {
    Write-Host ""
    Write-Host "  ⚠ 有异常码（$($last.Exc) 次）：从机返回了错误响应，查从机侧" -ForegroundColor Yellow
}
Write-Host ""
