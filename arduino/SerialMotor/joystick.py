import serial
import struct
import keyboard
import time

# ===============================
# 配置串口
# ===============================
SERIAL_PORT = "COM5"   # 修改为你的 Arduino COM 口
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# ===============================
# 控制参数
# ===============================
speed_hz = 2000          # 初始速度（Hz）
move_duration_ms = 200   # 每次按键移动的时间（毫秒）

print("方向键控制导轨：")
print("↑ ↓ 控制 Y 轴   | ← → 控制 X 轴")
print("+ - 调整速度    | ESC 退出程序")
print(f"初始速度: {speed_hz} Hz\n")

# ===============================
# 发送协议帧
# BF | motorMask | directionMask | speedHz | durationMs
# ===============================
def send_move(motor_id, direction, speed_hz, duration_ms):
    motorMask = (1 << motor_id)
    directionMask = (direction << motor_id)

    packet = struct.pack(
        "<BBBii",
        0xBF,               # header
        motorMask,
        directionMask,
        int(speed_hz),
        int(duration_ms)
    )

    ser.write(packet)
    print(f"Motor {motor_id} Dir:{direction} Speed:{speed_hz}Hz Duration:{duration_ms}ms")


# ===============================
# 主循环：键盘事件监听
# ===============================
try:
    while True:

        # --- X 轴控制 ---
        if keyboard.is_pressed('left'):
            send_move(0, 0, speed_hz, move_duration_ms)
            time.sleep(0.15)

        elif keyboard.is_pressed('right'):
            send_move(0, 1, speed_hz, move_duration_ms)
            time.sleep(0.15)

        # --- Y 轴控制 ---
        if keyboard.is_pressed('up'):
            send_move(1, 1, speed_hz, move_duration_ms)
            time.sleep(0.15)

        elif keyboard.is_pressed('down'):
            send_move(1, 0, speed_hz, move_duration_ms)
            time.sleep(0.15)

        # --- 速度增加 ---
        if keyboard.is_pressed('+') or keyboard.is_pressed('='):
            speed_hz += 200
            print(f"速度增加 → {speed_hz} Hz")
            time.sleep(0.2)

        # --- 速度减少 ---
        if keyboard.is_pressed('-'):
            speed_hz = max(200, speed_hz - 200)
            print(f"速度减少 → {speed_hz} Hz")
            time.sleep(0.2)

        # --- ESC 退出 ---
        if keyboard.is_pressed('esc'):
            print("退出程序")
            break

        time.sleep(0.01)

except KeyboardInterrupt:
    print("退出程序（Ctrl+C）")

finally:
    ser.close()
