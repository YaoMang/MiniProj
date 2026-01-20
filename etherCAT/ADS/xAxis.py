import pyads
import time

AMS_NET_ID = "5.40.xxx.xxx.1.1"   # 改成你自己的
IP = "127.0.0.1"                 # 本机或 PLC IP

plc = pyads.Connection(AMS_NET_ID, pyads.PORT_TC3PLC1, IP)
plc.open()

plc.write_by_name(
    "gAxisCmd_X.bEnable",
    True,
    pyads.PLCTYPE_BOOL
)

time.sleep(0.2)  # 给 PLC 至少一个 scan

plc.write_by_name("gAxisCmd_X.fPos", 100.0, pyads.PLCTYPE_LREAL)
plc.write_by_name("gAxisCmd_X.fVel", 250.0, pyads.PLCTYPE_LREAL)
plc.write_by_name("gAxisCmd_X.fAcc", 5000.0, pyads.PLCTYPE_LREAL)
plc.write_by_name("gAxisCmd_X.fDec", 5000.0, pyads.PLCTYPE_LREAL)

# 上升沿
plc.write_by_name("gAxisCmd_X.bMoveAbs", True, pyads.PLCTYPE_BOOL)
time.sleep(0.05)   # 不需要精确，> 1 scan 即可
plc.write_by_name("gAxisCmd_X.bMoveAbs", False, pyads.PLCTYPE_BOOL)

while True:
    busy = plc.read_by_name("gAxisCmd_X.bBusy", pyads.PLCTYPE_BOOL)
    done = plc.read_by_name("gAxisCmd_X.bDone", pyads.PLCTYPE_BOOL)
    pos  = plc.read_by_name("gAxisCmd_X.fActPos", pyads.PLCTYPE_LREAL)

    print(f"Busy={busy}, Done={done}, Pos={pos:.3f}")

    if done:
        break

    time.sleep(0.1)

err = plc.read_by_name("gAxisCmd_X.bError", pyads.PLCTYPE_BOOL)
if err:
    err_id = plc.read_by_name("gAxisCmd_X.nErrorID", pyads.PLCTYPE_UDINT)
    print("PLC Error ID:", err_id)

plc.close()
