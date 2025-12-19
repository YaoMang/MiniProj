from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(
    port='COM5', baudrate=115200,
    parity='N', stopbits=1, bytesize=8,
    timeout=0.5
)

if client.connect():
    print("✅ 串口连接成功，开始扫描地址...\n")
    for addr in range(1, 11):
        client.unit_id = addr
        try:
            result = client.read_holding_registers(address=0x0000, count=2)
            if hasattr(result, "registers"):
                print(f"✅ 发现设备: 地址 {addr}, 寄存器={result.registers}")
                break
            else:
                print(f"❌ 地址 {addr} 无响应")
        except Exception:
            pass
    client.close()
else:
    print("❌ 串口连接失败")
