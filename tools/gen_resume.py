# -*- coding: utf-8 -*-
"""生成程锦泉的嵌入式软件工程师简历（一页，含证件照）
   版式参考：项目介绍 / 主要工作 / 项目成果 三段式，技能与证书后置
"""
import os
from PIL import Image
from docx import Document
from docx.shared import Pt, Mm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

WORK = r"D:\MyProject\_resume_extract"
OUT_DOCX = r"D:\MyProject\程锦泉_嵌入式软件工程师_简历.docx"

BASE = 8.5          # 正文
LINE = 0.98         # 行距倍数
IND = 8             # 项目符号缩进
ACCENT = RGBColor(0x1F, 0x3B, 0x63)

# ---------- 证件照 ----------
im = Image.open(os.path.join(WORK, "photo.jpg")).convert("RGB")
W, H = im.size
ratio = 25.0 / 35.0
if W / H > ratio:
    nw = int(H * ratio); im = im.crop(((W - nw) // 2, 0, (W - nw) // 2 + nw, H))
else:
    nh = int(W / ratio); top = int(H * 0.04)
    if top + nh > H: top = H - nh
    im = im.crop((0, top, W, top + nh))
face = os.path.join(WORK, "photo_1inch.jpg")
im.resize((300, 420), Image.LANCZOS).save(face, quality=95)

# ---------- 文档 ----------
doc = Document()
for s in doc.sections:
    s.top_margin = Mm(9); s.bottom_margin = Mm(7)
    s.left_margin = Mm(12); s.right_margin = Mm(12)

st = doc.styles["Normal"]
st.font.name = "微软雅黑"; st.font.size = Pt(BASE)
st._element.rPr.rFonts.set(qn("w:eastAsia"), "微软雅黑")
st.paragraph_format.space_before = Pt(0)
st.paragraph_format.space_after = Pt(0)
st.paragraph_format.line_spacing_rule = WD_LINE_SPACING.MULTIPLE
st.paragraph_format.line_spacing = LINE


def setf(run, size=BASE, bold=False, color=None):
    run.font.size = Pt(size); run.font.bold = bold
    run.font.name = "微软雅黑"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "微软雅黑")
    if color is not None: run.font.color.rgb = color


def rich(parts, size=BASE, indent=0, hanging=0, before=0, after=0.5, align=None):
    p = doc.add_paragraph()
    if align is not None: p.alignment = align
    p.paragraph_format.space_before = Pt(before)
    p.paragraph_format.space_after = Pt(after)
    p.paragraph_format.line_spacing = LINE
    if indent: p.paragraph_format.left_indent = Pt(indent)
    if hanging: p.paragraph_format.first_line_indent = Pt(-hanging)
    for t, b, c in parts:
        setf(p.add_run(t), size, b, c)
    return p


def marked(text, size=BASE, indent=0, hanging=0, before=0, after=0.5):
    """「...」→ 加粗"""
    parts = []
    for i, seg in enumerate(text.split("「")):
        if i == 0:
            if seg: parts.append((seg, False, None))
            continue
        if "」" in seg:
            b, rest = seg.split("」", 1)
            parts.append((b, True, None))
            if rest: parts.append((rest, False, None))
        else:
            parts.append(("「" + seg, False, None))
    return rich(parts, size=size, indent=indent, hanging=hanging, before=before, after=after)


def bullet(text, size=BASE, mark=False):
    fn = marked if mark else rich
    if mark:
        return fn("· " + text, size=size, indent=IND, hanging=IND)
    return rich([("· ", False, None), (text, False, None)],
                size=size, indent=IND, hanging=IND)


def arrow(label, text, size=8):
    """▸ 项目介绍 / 主要工作 / 项目成果 这一行的标签"""
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(0.5)
    p.paragraph_format.space_after = Pt(0.5)
    p.paragraph_format.line_spacing = LINE
    setf(p.add_run("▸ "), size, True, ACCENT)
    setf(p.add_run(label), size, True, None)
    return p


def section(title):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(1)
    p.paragraph_format.line_spacing = LINE
    setf(p.add_run(title), 10, True, ACCENT)
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr"); b = OxmlElement("w:bottom")
    b.set(qn("w:val"), "single"); b.set(qn("w:sz"), "6")
    b.set(qn("w:space"), "1"); b.set(qn("w:color"), "1F3B63")
    pbdr.append(b); pPr.append(pbdr)


# ---------- 抬头 ----------
tbl = doc.add_table(rows=1, cols=2)
tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
tbl.autofit = False
tbl.columns[0].width = Mm(150); tbl.columns[1].width = Mm(26)
left, right = tbl.cell(0, 0), tbl.cell(0, 1)
left.width = Mm(150); right.width = Mm(26)
for c in (left, right):
    tcPr = c._tc.get_or_add_tcPr(); mar = OxmlElement("w:tcMar")
    for side in ("top", "bottom", "start", "end"):
        el = OxmlElement(f"w:{side}"); el.set(qn("w:w"), "0"); el.set(qn("w:type"), "dxa")
        mar.append(el)
    tcPr.append(mar)


def cp(cell, text, size=BASE, bold=False, color=None, first=False, after=0.5):
    p = cell.paragraphs[0] if first else cell.add_paragraph()
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(after)
    p.paragraph_format.line_spacing = LINE
    if text: setf(p.add_run(text), size, bold, color)


cp(left, "程  锦  泉", 16, True, ACCENT, first=True, after=1.5)
cp(left, "求职意向：嵌入式软件工程师（MCU / RTOS / 工业通信方向）", 9, True)
cp(left, "电话 17763418416　|　邮箱 771594198@qq.com　|　广东第二师范学院 · 电子信息工程（本科）· 2026 届", 8.5)
rp = right.paragraphs[0]
rp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
rp.paragraph_format.space_before = Pt(0); rp.paragraph_format.space_after = Pt(0)
rp.add_run().add_picture(face, width=Mm(22))

# ---------- 教育背景 ----------
section("教育背景")
rich([("广东第二师范学院", True, None),
      ("　　电子信息工程（本科）　　2022.09 – 2026.06", False, None)], size=9)
rich([("核心课程：C 语言、嵌入式软件开发、嵌入式 Linux 应用开发、微处理器设计、"
       "单片机原理与应用、数字信号处理、计算机网络、EDA 技术", False, None)], size=8)

# ---------- 实习经历 ----------
section("实习经历")
rich([("广州平谐网络服务有限公司", True, None),
      ("　　嵌入式软件助理工程师　　2025.06 – 2026.01", False, None)], size=9)
bullet("参与基于 STM32 的工业物联网终端开发，负责传感器数据采集、通信模块开发与系统联调；"
       "基于 FreeRTOS 完成采集 / 通信 / 数据处理多任务架构，调试 UART、I2C、ADC 外设驱动。")
bullet("参与 RS485 Modbus RTU 从站协议实现；用串口工具与逻辑分析仪逐层排查 CRC 错误、"
       "通信无响应等问题，总结出「物理层 → 协议层 → 应用层」的分层定位方法。", mark=True)

# ---------- 项目经历 ----------
section("项目经历")

# ===== 项目一 =====
rich([("双 MCU 工业网关 | RS485 振动监测与端侧 AI 异常识别平台", True, ACCENT),
      ("　　个人项目", False, None),
      ("　　2026.04 – 2026.07", True, None)], size=9.5, before=0.5, after=0.5)
marked("「项目介绍」　以 STM32F411 为采集节点、ESP32-S3 为边缘网关构建「双 MCU 架构」，"
       "实现振动数据的实时采集、有线传输、端侧推理与本地显示完整链路：采集端以硬件定时器"
       "严格 50 Hz 采样 MPU6050 六轴加速度并维护采样窗口，经 RS485 对外提供原始波形与特征；"
       "网关端完成窗口读取、端侧推理与本地界面显示。",
       size=8, indent=IND, hanging=IND, after=0.5)
marked("「主要工作」", size=8, indent=IND, hanging=IND, after=0.5)
for t in [
    "采集端以「硬件定时器严格 50 Hz 采样」MPU6050 六轴加速度，采用「双缓冲机制」维护 "
    "100 样本 × 6 轴的采样窗口，并提取时域统计特征经 RS485 对外提供。",
    "网关端基于「自研 Modbus RTU 主机协议栈」完成窗口头、600 点原始波形与特征寄存器的"
    "分块读取，结合序号一致性校验与重试机制保证跨窗口数据拼接的正确性；读取结果送入"
    "「TFLite Micro 端侧推理模型」完成异常识别，并通过 LVGL 构建数据 / 波形 / 诊断三页"
    "本地界面与 MQTT 上云。",
    "调试中发现「总线首试成功率仅 58%」且“首试必失败、重试必成功”；按电气方向排查"
    "（降低波特率、加装 120 Ω 终端电阻、放宽超时）「均被反证」后，「自研双串口增量对账脚本」"
    "以主从计数器增量比对定位丢帧方向，测得主站发送 176 次而从机仅收到 100 次请求，据此将"
    "问题锁定在请求方向；最终确认根因为从机在 TC 标志置位前无法重新武装接收，主站事务间隔"
    "不足导致整帧被静默丢弃，标定后将事务间隔固定为 400 ms。",
]:
    bullet(t, size=8, mark=True)
marked("「项目成果」　「首试成功率由 58% 提升至 100%」，经「4 小时长跑验证 47510 次事务"
       "首试成功率 100%」，超时、重试与 CRC 错误均为 0，内部堆净变化为 0。",
       size=8, indent=IND, hanging=IND, after=0.5)
rich([("技术栈：STM32F411、ESP32-S3、FreeRTOS（双核任务划分）、RS485、Modbus RTU、"
       "TFLite Micro、LVGL、MQTT、C", False, None)], size=8, indent=IND, hanging=IND)
rich([("应用场景：电机 / 风机振动监测、预测性维护、工业设备状态采集、边缘 AI 异常识别",
       False, None)], size=8, indent=IND, hanging=IND)

# ===== 项目二 =====
rich([("端云协同声纹门禁系统 | 离线语音触发与云端声纹验证", True, ACCENT),
      ("　　毕业设计", False, None),
      ("　　2026.01 – 2026.04", True, None)], size=9.5, before=2, after=0.5)
marked("「项目介绍」　以 STM32F411RET6 为主控，「集成语音采集模块、OLED 显示屏、舵机以及 "
       "ESP8266」，实现语音采集、网络传输、服务器判定及门锁控制完整流程。",
       size=8, indent=IND, hanging=IND, after=0.5)
marked("「主要工作」", size=8, indent=IND, hanging=IND, after=0.5)
for t in [
    "通过 ADC 采集语音信号，首先计算环境噪声基线，在限定时间窗口内利用「连续阈值检测完成"
    "语音触发」；触发后采集固定长度数据窗口，并提取均值、最大值、最小值、峰峰值、能量及 "
    "8 段分布等「13 维轻量级统计特征」。",
    "通过 USART 控制 ESP8266 执行 AT 指令，完成 WiFi 连接及「TCP 服务器建链」，将提取的"
    "特征上传至 Python 服务器进行判定，并根据 PASS/FAIL 结果控制舵机开锁；设计 WiFi 及"
    "服务器连接状态管理逻辑，结合 UART 日志与 OLED 状态显示完成异常状态调试。",
    "针对「端侧算力与采样资源受限、终端只能上传轻量级统计特征而无法上传完整音频流」的限制，"
    "接入「讯飞声纹识别 WebAPI 完成 1:1 声纹比对」（阈值 0.80）作为云端扩展路径，形成"
    "「本地轻量粗筛 + 云端精确判定的两级验证架构」。",
]:
    bullet(t, size=8, mark=True)
marked("「项目成果」　完成 WiFi 连接、TCP 通信、本人语音开锁、他人语音拒识、静默超时保护"
       "全流程联调；对比两条识别路径的准确率与响应速度；毕业论文成绩「83.3 分」。",
       size=8, indent=IND, hanging=IND, after=0.5)
rich([("技术栈：STM32F411RET6、ADC、ESP8266、TCP Socket、Python、讯飞声纹 WebAPI、OLED、"
       "PWM 舵机、C", False, None)], size=8, indent=IND, hanging=IND)
rich([("应用场景：智能门禁、语音身份验证、端云协同 IoT 终端、低成本联网控制设备",
       False, None)], size=8, indent=IND, hanging=IND)

# ---------- 专业技能与证书荣誉（后置） ----------
section("专业技能与证书荣誉")
rich([("嵌入式开发：STM32F1/F4、Cortex-M3/M4、C 语言、HAL 库、FreeRTOS、LVGL、"
       "TFLite Micro、多任务架构、栈与内存调优", False, None)], size=8)
rich([("通信协议：UART、I2C、SPI、ADC、PWM、RS485、Modbus RTU、CRC16、自定义二进制帧、"
       "TCP Socket、MQTT　　工具与调试：Keil、STM32CubeMX、PlatformIO、ESP-IDF、"
       "串口调试、逻辑分析仪、万用表、原理图阅读", False, None)], size=8)
rich([("证书与英语：大学英语六级（CET-6，2024.12）　·　初级嵌入式软件开发工程师",
       False, None)], size=8)

doc.save(OUT_DOCX)
print("已生成:", OUT_DOCX)
