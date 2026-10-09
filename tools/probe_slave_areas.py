import serial, time
def crc16(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8):
            c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def frame(addr,fc,start,qty):
    p=bytes([addr,fc,start>>8,start&0xFF,qty>>8,qty&0xFF])
    c=crc16(p)
    return p+bytes([c&0xFF,c>>8])
def read_regs(ser,start,qty,label):
    ser.reset_input_buffer()
    f=frame(1,3,start,qty)
    ser.write(f); ser.flush()
    time.sleep(0.15)
    r=ser.read(300)
    ok = len(r)>=5 and r[1]==3
    print(f"  {label:28s} 请求={f.hex(' ')}  应答={r.hex(' ') if r else '(无)'}")
    return r
s=serial.Serial('COM8',115200,timeout=0.3)
time.sleep(0.3)
print("=== 直接用 F411 从机测各地址区（FC03）===")
read_regs(s,0x0000,4,  "A区 0x0000 x4  (慢车道起点)")
read_regs(s,0x0000,28, "A区 0x0000 x28 (慢车道全量)")
read_regs(s,0x0100,4,  "D区 0x0100 x4  (窗口头!)")
read_regs(s,0x0101,1,  "D区 0x0101 x1  (序号)")
read_regs(s,0x0104,4,  "D区 0x0104 x4  (窗口数据)")
read_regs(s,0x0400,4,  "E区 0x0400 x4  (特征)")
read_regs(s,0x0020,4,  "C区 0x0020 x4  (控制)")
s.close()