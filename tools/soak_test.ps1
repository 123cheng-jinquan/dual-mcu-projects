# =============================================================================
#  soak_test.ps1 — 长时间稳定性测试（默认 4 小时）
# =============================================================================
#  目的：验证"长时间连续运行"下的稳定性，这是 §D11 的验收项。
#  只看单次心跳是不够的 —— 那只能说明"此刻没问题"。长跑要回答的是：
#    ① 内存有没有缓慢泄漏（FreeHeap 是否单调下降）？
#    ② 队列有没有越积越多（深度是否增长）？
#    ③ 485 错误率在长时间尺度上是否稳定（还是随时间恶化）？
#    ④ 有没有看门狗复位 / panic / 偶发掉线？
#    ⑤ 推理有没有卡住或变慢？
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\soak_test.ps1
#    powershell -ExecutionPolicy Bypass -File tools\soak_test.ps1 -Hours 0.1      # 6 分钟试跑
#    powershell -ExecutionPolicy Bypass -File tools\soak_test.ps1 -Hours 4 -Master COM6 -Slave COM8
#
#  输出（都在 logs/ 下）：
#    soak_<时间戳>_samples.csv   周期性采样（可直接用 Excel 画趋势图）
#    soak_<时间戳>_master.log    ESP32 原始日志
#    soak_<时间戳>_slave.log     F411 原始日志
#    soak_<时间戳>_report.txt    最终报告（结论 + 趋势）
#
#  ⚠ 注意：
#   · 测试期间**不要重新烧写固件**，会打断测试且数据作废
#   · 脚本不复位设备（DTR/RTS 保持不动作），监测的是"当前离线运行状态"
#   · 两个串口要独占，跑之前先关掉 pio device monitor 和串口助手
# =============================================================================

param(
    [double] $Hours      = 4,
    [string] $Master     = 'COM6',
    [int]    $MasterBaud = 115200,
    [string] $Slave      = 'COM8',
    [int]    $SlaveBaud  = 115200,
    [int]    $SampleSec  = 30,          # 每多少秒落一行采样
    [string] $OutDir     = 'logs'
)

$ErrorActionPreference = 'Stop'
$stamp   = Get-Date -Format 'yyyyMMdd_HHmmss'
$root    = Split-Path $PSScriptRoot -Parent
$dir     = Join-Path $root $OutDir
New-Item -ItemType Directory -Force -Path $dir | Out-Null

$csvPath    = Join-Path $dir "soak_${stamp}_samples.csv"
$masterPath = Join-Path $dir "soak_${stamp}_master.log"
$slavePath  = Join-Path $dir "soak_${stamp}_slave.log"
$reportPath = Join-Path $dir "soak_${stamp}_report.txt"

$totalSec = [int]($Hours * 3600)

Write-Host ""
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "  长跑稳定性测试" -ForegroundColor Cyan
Write-Host "═══════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host ("  时长     : {0} 小时（{1} 秒）" -f $Hours, $totalSec)
Write-Host ("  主站     : {0} @ {1}" -f $Master, $MasterBaud)
Write-Host ("  从站     : {0} @ {1}" -f $Slave,  $SlaveBaud)
Write-Host ("  采样周期 : {0} 秒" -f $SampleSec)
Write-Host ("  输出目录 : {0}" -f $dir)
Write-Host ""
Write-Host "  ⚠ 测试期间不要重新烧写固件；先关掉串口助手/监视器" -ForegroundColor Yellow
Write-Host ""

# ---- 打开两个串口（不复位设备）----
function Open-Port($name, $baud) {
    $sp = New-Object System.IO.Ports.SerialPort $name, $baud, 'None', 8, 'One'
    $sp.ReadTimeout = 50
    $sp.DtrEnable   = $false      # 关键：不拉低 DTR/RTS，避免复位 ESP32
    $sp.RtsEnable   = $false
    try { $sp.Open() } catch {
        Write-Host ("  ✗ 打不开 {0}：{1}" -f $name, $_.Exception.Message) -ForegroundColor Red
        Write-Host "    （是不是被 pio device monitor 或串口助手占着？）" -ForegroundColor Yellow
        return $null
    }
    Start-Sleep -Milliseconds 200
    try { $sp.DiscardInBuffer() } catch { }
    return $sp
}

$pm = Open-Port $Master $MasterBaud
$ps = Open-Port $Slave  $SlaveBaud
if ($null -eq $pm) { exit 1 }
if ($null -eq $ps) { $pm.Close(); exit 1 }

# ---- 采样行缓冲（保存最近的原文，用于解析"最新值"）----
$bufM = New-Object System.Text.StringBuilder
$bufS = New-Object System.Text.StringBuilder
$logM = New-Object System.IO.StreamWriter($masterPath, $false, [System.Text.UTF8Encoding]::new($false))
$logS = New-Object System.IO.StreamWriter($slavePath,  $false, [System.Text.UTF8Encoding]::new($false))
$samples = New-Object System.Collections.Generic.List[object]

# ---- 把缓冲区里的新文本读进来 ----
function Pump($port, $sb, $writer) {
    $buf = New-Object byte[] 8192
    for ($i = 0; $i -lt 40; $i++) {          # 每次 Pump 最多读 40 轮，避免长时间不采样
        try {
            $n = $port.Read($buf, 0, $buf.Length)
            if ($n -le 0) { break }
            $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
            [void]$sb.Append($txt)
            $writer.Write($txt)
        } catch { break }
    }
    $writer.Flush()
    # 缓冲只留尾部 64KB（长跑几小时会攒下几十 MB）
    if ($sb.Length -gt 65536) { [void]$sb.Remove(0, $sb.Length - 32768) }
}

# ---- 解析主站最新心跳与内存 ----
function ParseMaster($text) {
    $r = [ordered]@{}
    $m = [regex]::Matches($text, 'TX\s+(\d+)\s+RX\s+(\d+)\s+超时\s+(\d+)\s+重试\s+(\d+)\s+异常\s+(\d+)\s+CRC错\s+(\d+)')
    if ($m.Count -gt 0) {
        $l = $m[$m.Count-1]
        $r.TX=[int]$l.Groups[1].Value; $r.RX=[int]$l.Groups[2].Value
        $r.Timeout=[int]$l.Groups[3].Value; $r.Retry=[int]$l.Groups[4].Value
        $r.Exception=[int]$l.Groups[5].Value; $r.CrcErr=[int]$l.Groups[6].Value
    }
    $q = [regex]::Matches($text, '等待处理\s+(\d+)')
    if ($q.Count -gt 0) { $r.QueueDepth = [int]$q[$q.Count-1].Groups[1].Value }
    $h = [regex]::Matches($text, '内部堆剩余\s+(\d+)')
    if ($h.Count -gt 0) { $r.FreeHeap = [int]$h[$h.Count-1].Groups[1].Value }
    $r.Resets  = ([regex]::Matches($text, 'rst:0x')).Count
    $r.Panics  = ([regex]::Matches($text, 'Guru Meditation')).Count
    $r.WdtHits = ([regex]::Matches($text, 'task_wdt')).Count
    $r.Offline = ([regex]::Matches($text, '节点离线')).Count
    return $r
}

# ---- 解析从站 [mb] 行与 [hb] 行 ----
function ParseSlave($text) {
    $r = [ordered]@{}
    $m = [regex]::Matches($text, '\[mb\]\s+rx=(\d+)\(0len=(\d+)\s+drop=(\d+)\s+long=(\d+)\s+(\d+)B\)\s+parse0=(\d+)\s+tx=(\d+)\((\d+)B\)')
    if ($m.Count -gt 0) {
        $l = $m[$m.Count-1]
        $r.RxEvents=[int]$l.Groups[1].Value; $r.RxSize0=[int]$l.Groups[2].Value
        $r.RxDrop=[int]$l.Groups[3].Value;   $r.RxLong=[int]$l.Groups[4].Value
        $r.RxBytes=[int]$l.Groups[5].Value;  $r.Parse0=[int]$l.Groups[6].Value
        $r.TxFrames=[int]$l.Groups[7].Value; $r.TxBytes=[int]$l.Groups[8].Value
    }
    $t = [regex]::Matches($text, '\[hb\]\s+t=(\d+)s')
    if ($t.Count -gt 0) { $r.UptimeS = [int]$t[$t.Count-1].Groups[1].Value }
    $r.Resets = ([regex]::Matches($text, 'rst:')).Count
    return $r
}

# ---- 主循环 ----
#  ★ 内存怎么取 ★
#    ESP32 的**周期日志里没有内存字段**（只有心跳/诊断/推理），
#    内存只在串口命令 `status` 的输出里（见 console.cpp 的 print_status）。
#    所以脚本**周期性往 COM6 发一条 `status`**，把回显解析出来。
#    `status` 是纯只读快照（不碰总线、不改状态），对长跑没有干扰 ——
#    好处是**不用改固件**就能拿到内存趋势，长跑跑完还能照常烧新固件。
$STATUS_EVERY = [Math]::Max(1, [int](300 / $SampleSec))   # 约每 5 分钟问一次

# ---- 定义 status 询问函数（开机先问一次，之后周期问）----
function Ask-Status($pm, $bufM, $logM) {
    try {
        try { $pm.DiscardInBuffer() } catch { }
        $cmd = [System.Text.Encoding]::ASCII.GetBytes("status`r`n")
        $pm.Write($cmd, 0, $cmd.Length)
        Start-Sleep -Milliseconds 1500
        $probe = New-Object byte[] 4096
        $got = 0
        $t1 = [System.Diagnostics.Stopwatch]::StartNew()
        while ($t1.ElapsedMilliseconds -lt 800) {
            try {
                $n = $pm.Read($probe, 0, $probe.Length)
                if ($n -gt 0) {
                    $got += $n
                    $txt = [System.Text.Encoding]::UTF8.GetString($probe, 0, $n)
                    [void]$bufM.Append($txt)
                    $logM.Write($txt)
                }
            } catch { break }
        }
        $logM.Flush()
        Write-Host ("      [status] 收到 {0} 字节" -f $got) -ForegroundColor DarkGray
    } catch {
        Write-Host ("      [status] 发送失败: {0}" -f $_.Exception.Message) -ForegroundColor Red
    }
}

# 开机先问一次：这样**第一个采样点就带内存基线**，
# 短测试也能验出内存字段是否正常（不必等到 5 分钟后）。
Write-Host "  取基线（发一条 status）..." -ForegroundColor DarkGray
Ask-Status $pm $bufM $logM

$sw      = [System.Diagnostics.Stopwatch]::StartNew()
$next    = 0
$sampleN = 0
$lastPct = -1

try {
    while ($sw.Elapsed.TotalSeconds -lt $totalSec) {
        Pump $pm $bufM $logM
        Pump $ps $bufS $logS

        if ($sw.Elapsed.TotalSeconds -ge $next) {
            $next += $SampleSec
            $sampleN++

            # 周期性问一次 status（取内存/队列）；基线已在开机时取过
            if (($sampleN % $STATUS_EVERY) -eq 0) {
                Ask-Status $pm $bufM $logM
            }

            $m = ParseMaster $bufM.ToString()
            $s = ParseSlave  $bufS.ToString()

            $row = [ordered]@{ 时间 = (Get-Date -Format 'HH:mm:ss'); 已运行秒 = [int]$sw.Elapsed.TotalSeconds }
            foreach ($k in $m.Keys) { $row["M_$k"] = $m[$k] }
            foreach ($k in $s.Keys) { $row["S_$k"] = $s[$k] }
            $samples.Add([pscustomobject]$row)

            # 进度提示（每 ~1% 或每 20 个采样打一次）
            $pct = [int](100 * $sw.Elapsed.TotalSeconds / $totalSec)
            if ($pct -ne $lastPct -and ($pct % 2 -eq 0)) {
                $lastPct = $pct
                $tx = if ($m.Contains('TX')) { $m.TX } else { 0 }
                $to = if ($m.Contains('Timeout')) { $m.Timeout } else { 0 }
                $rt = if ($m.Contains('Retry')) { $m.Retry } else { 0 }
                $hp = if ($m.Contains('FreeHeap')) { $m.FreeHeap } else { 0 }
                $qd = if ($m.Contains('QueueDepth')) { $m.QueueDepth } else { 0 }
                Write-Host ("  [{0,3}%] {1,7}s  TX={2,-6} 超时={3,-5} 重试={4,-5} 堆={5,-7} 队列={6}" -f `
                            $pct, [int]$sw.Elapsed.TotalSeconds, $tx, $to, $rt, $hp, $qd)
            }
            if ($sampleN % 20 -eq 0) {
                $csv = $samples | ConvertTo-Csv -NoTypeInformation
                [System.IO.File]::WriteAllLines($csvPath, $csv, [System.Text.UTF8Encoding]::new($false))
            }
        } else {
            Start-Sleep -Milliseconds 200
        }
    }
} finally {
    Pump $pm $bufM $logM
    Pump $ps $bufS $logS
    $pm.Close(); $ps.Close()
    $logM.Close(); $logS.Close()
}

$csv = $samples | ConvertTo-Csv -NoTypeInformation
[System.IO.File]::WriteAllLines($csvPath, $csv, [System.Text.UTF8Encoding]::new($false))

# =============================================================================
#  报告
# =============================================================================
$first = $samples[0]
$last  = $samples[$samples.Count-1]

$lines = New-Object System.Collections.Generic.List[string]
function L($t) { $lines.Add($t) }

L "长跑稳定性测试报告"
L ("测试时间 : {0}  ~  {1}" -f (Get-Date).AddSeconds(-$totalSec).ToString('yyyy-MM-dd HH:mm:ss'), (Get-Date).ToString('yyyy-MM-dd HH:mm:ss'))
L ("时长     : {0} 小时（{1} 个采样点，每 {2} 秒一个）" -f $Hours, $samples.Count, $SampleSec)
L ("主站/从站: {0} / {1}" -f $Master, $Slave)
L ""
L "──────────────── 485 总线 ────────────────"
$dTx = $last.M_TX - $first.M_TX
$dRx = $last.M_RX - $first.M_RX
$dTo = $last.M_Timeout - $first.M_Timeout
$dRt = $last.M_Retry - $first.M_Retry
$dCr = $last.M_CrcErr - $first.M_CrcErr
$dEx = $last.M_Exception - $first.M_Exception
L ("  事务总数        : {0}" -f $dTx)
L ("  成功            : {0}" -f $dRx)
L ("  超时            : {0}" -f $dTo)
L ("  重试            : {0}" -f $dRt)
L ("  CRC 错          : {0}" -f $dCr)
L ("  异常码          : {0}" -f $dEx)
if ($dTx -gt 0) {
    L ("  ★ 首试成功率    : {0:N2}%" -f (100.0 * ($dTx - $dTo) / $dTx))
}
L ""
L "──────────────── 稳定性 ────────────────"
L ("  ESP32 复位      : {0}" -f $last.M_Resets)
L ("  ESP32 panic     : {0}" -f $last.M_Panics)
L ("  看门狗告警      : {0}" -f $last.M_WdtHits)
L ("  节点离线事件    : {0}" -f $last.M_Offline)
L ("  F411 复位       : {0}" -f $last.S_Resets)
L ("  F411 队列溢出   : {0}（起 {1} → 终 {2}）" -f ($last.S_RxDrop - $first.S_RxDrop), $first.S_RxDrop, $last.S_RxDrop)
L ("  F411 静默丢弃   : {0}（起 {1} → 终 {2}）" -f ($last.S_Parse0 - $first.S_Parse0), $first.S_Parse0, $last.S_Parse0)
L ""
L "──────────────── 内存趋势（重点：泄漏会让它单调下降）────────────────"
$heapStart = $first.M_FreeHeap
$heapEnd   = $last.M_FreeHeap
$heapMin   = ($samples | ForEach-Object { $_.M_FreeHeap } | Where-Object { $_ -gt 0 } | Measure-Object -Minimum).Minimum
L ("  起始            : {0} 字节" -f $heapStart)
L ("  结束            : {0} 字节" -f $heapEnd)
L ("  期间最小值      : {0} 字节" -f $heapMin)
if ($heapStart -gt 0) {
    $leak = $heapStart - $heapEnd
    L ("  净变化          : {0} 字节（{1}）" -f $leak, $(if ($leak -gt 0) { "减少" } else { "增加" }))
    L ("  每小时变化      : {0:N0} 字节/小时" -f ($leak / $Hours))
    if ($leak -gt 4096) {
        L "  ⚠ 疑似泄漏：净减少超过 4KB。看下 CSV 里 M_FreeHeap 是不是单调下降。"
    } else {
        L "  ✓ 无明显泄漏（净变化 < 4KB，属于正常的碎片/缓存波动）"
    }
}
L ""
L "──────────────── 队列深度（重点：持续增长说明消费者跟不上）────────────────"
$qd = $samples | ForEach-Object { $_.M_QueueDepth }
$qdMax = ($qd | Measure-Object -Maximum).Maximum
$qdAvg = ($qd | Measure-Object -Average).Average
L ("  起始 / 结束     : {0} / {1}" -f $first.M_QueueDepth, $last.M_QueueDepth)
L ("  平均 / 最大     : {0:N2} / {1}" -f $qdAvg, $qdMax)
if ($qdMax -le 3) {
    L "  ✓ 队列一直很浅，生产-消费平衡"
} else {
    L "  ⚠ 队列曾经堆到 $qdMax，检查是否有请求周期性打满"
}
L ""
L "──────────────── 文件 ────────────────"
L ("  采样 CSV : {0}" -f $csvPath)
L ("  主站日志 : {0}" -f $masterPath)
L ("  从站日志 : {0}" -f $slavePath)

[System.IO.File]::WriteAllLines($reportPath, $lines, [System.Text.UTF8Encoding]::new($false))
$lines | ForEach-Object { Write-Host $_ }
Write-Host ""
Write-Host ("报告已保存: {0}" -f $reportPath) -ForegroundColor Green
