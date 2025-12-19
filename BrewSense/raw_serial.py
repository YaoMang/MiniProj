#import serial
#import struct
#import time
#
#def crc16_modbus(data: bytes) -> int:
#    crc = 0xFFFF
#    for b in data:
#        crc ^= b
#        for _ in range(8):
#            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
#    return crc
#
#PORT = "COM5"      # ← 改为你的端口号
#BAUD = 115200
#REQ = bytes.fromhex("01 03 03 80 00 06 C4 64")  # 读角度+圈数+状态+速度
#
#with serial.Serial(PORT, BAUD, bytesize=8, parity=serial.PARITY_NONE,
#                   stopbits=serial.STOPBITS_ONE, timeout=0.2) as ser:
#    ser.reset_input_buffer()
#    ser.write(REQ)
#    ser.flush()
#    time.sleep(0.05)
#    resp = ser.read(64)
#    print("Raw:", resp.hex(" "))
#
#    if len(resp) >= 17 and resp[0] == 1 and resp[1] == 3:
#        data = resp[3:-2]
#        # 按文档：角度(4 B) + 圈数(4 B) + 状态(2 B) + 速度(2 B)
#        angle = int.from_bytes(data[0:4], "big", signed=False)
#        turns = int.from_bytes(data[4:8], "big", signed=True)
#        status = int.from_bytes(data[8:10], "big", signed=False)
#        speed_raw = int.from_bytes(data[10:12], "big", signed=True)
#        speed_rpm = speed_raw * 600000 / 2097152  # 21 位分辨率
#        angle_deg = angle * 360 / 2097152
#        print(f"Angle: {angle} → {angle_deg:.4f}°")
#        print(f"Turns: {turns}")
#        print(f"Speed: {speed_raw} → {speed_rpm:.3f} rpm")
#        print(f"Status: 0x{status:04X}")
#    else:
#        print("Invalid / no response")
#

import serial
import time

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

PORT = "COM5"      # ← 改成你的端口号
BAUD = 115200
REQ = bytes.fromhex("01 03 03 80 00 06 C4 64")  # 读角度+圈数+状态+速度

def parse_response(resp: bytes):
    if len(resp) >= 17 and resp[0] == 1 and resp[1] == 3:
        data = resp[3:-2]
        # 角度(4B) + 圈数(4B) + 状态(2B) + 速度(2B)
        angle = int.from_bytes(data[0:4], "big", signed=False)
        turns = int.from_bytes(data[4:8], "big", signed=True)
        status = int.from_bytes(data[8:10], "big", signed=False)
        speed_raw = int.from_bytes(data[10:12], "big", signed=True)

        angle_deg = angle * 360 / 2097152       # 单圈 21 位分辨率
        speed_rpm = speed_raw * 600000 / 2097152

        print(f"Angle: {angle_deg:9.4f}°  |  Turns: {turns:5d}  |  "
              f"Speed: {speed_rpm:8.3f} rpm  |  Status: 0x{status:04X}")
        return True
    return False

with serial.Serial(PORT, BAUD, bytesize=8, parity=serial.PARITY_NONE,
                   stopbits=serial.STOPBITS_ONE, timeout=0.2) as ser:
    print(f"Listening on {PORT} @ {BAUD} baud...")
    while True:
        ser.reset_input_buffer()
        ser.write(REQ)
        ser.flush()
        time.sleep(0.003)               # 帧间隔约 3 ms，说明书建议 350–1000 µs
        resp = ser.read(64)

        if not resp:
            print("No response")
        else:
            if not parse_response(resp):
                print("Invalid / CRC error:", resp.hex(" "))

        time.sleep(0.1)                 # 读取周期 100 ms，可自行调整
