import time
import pyads

AMS_NET_ID = "5.1.204.160.1.1"   # 改成你的
AMS_PORT   = pyads.PORT_TC3PLC1

PLC_IP     = "192.168.1.10"      # 可选，用于 Route

# -----------------------------
# ADS 连接
# -----------------------------
plc = pyads.Connection(AMS_NET_ID, AMS_PORT, PLC_IP)
plc.open()

# -----------------------------
# 工具函数
# -----------------------------
def wait_until(expr, timeout=5.0, period=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if expr():
            return True
        time.sleep(period)
    return False


def axis_enable(axis):
    plc.write_by_name(f"{axis}.bEnable", True, pyads.PLCTYPE_BOOL)
    ok = wait_until(lambda: plc.read_by_name(f"{axis}.bReady", pyads.PLCTYPE_BOOL))
    print(f"[{axis}] Enable -> Ready:", ok)
    return ok


def axis_reset(axis):
    plc.write_by_name(f"{axis}.bReset", True, pyads.PLCTYPE_BOOL)
    time.sleep(0.1)
    plc.write_by_name(f"{axis}.bReset", False, pyads.PLCTYPE_BOOL)
    print(f"[{axis}] Reset sent")


def axis_move_abs(axis, pos, vel=50.0, acc=200.0, dec=200.0):
    plc.write_by_name(f"{axis}.fPos", pos, pyads.PLCTYPE_LREAL)
    plc.write_by_name(f"{axis}.fVel", vel, pyads.PLCTYPE_LREAL)
    plc.write_by_name(f"{axis}.fAcc", acc, pyads.PLCTYPE_LREAL)
    plc.write_by_name(f"{axis}.fDec", dec, pyads.PLCTYPE_LREAL)

    plc.write_by_name(f"{axis}.bMoveAbs", True, pyads.PLCTYPE_BOOL)
    time.sleep(0.05)
    plc.write_by_name(f"{axis}.bMoveAbs", False, pyads.PLCTYPE_BOOL)

    print(f"[{axis}] MoveAbs -> {pos}")

    wait_until(lambda: plc.read_by_name(f"{axis}.bBusy", pyads.PLCTYPE_BOOL))
    wait_until(lambda: plc.read_by_name(f"{axis}.bDone", pyads.PLCTYPE_BOOL))

    act = plc.read_by_name(f"{axis}.fActPos", pyads.PLCTYPE_LREAL)
    print(f"[{axis}] Done, ActPos = {act:.3f}")


def axis_stop(axis):
    plc.write_by_name(f"{axis}.bStop", True, pyads.PLCTYPE_BOOL)
    time.sleep(0.1)
    plc.write_by_name(f"{axis}.bStop", False, pyads.PLCTYPE_BOOL)
    print(f"[{axis}] Stop sent")


def axis_check_error(axis):
    err = plc.read_by_name(f"{axis}.bError", pyads.PLCTYPE_BOOL)
    if err:
        code = plc.read_by_name(f"{axis}.nErrorID", pyads.PLCTYPE_UDINT)
        print(f"[{axis}] ERROR! Code = {code}")
    return err


# -----------------------------
# 测试流程
# -----------------------------
try:
    # X 轴测试
    axis_enable("gAxisCmd_X")
    axis_move_abs("gAxisCmd_X", 100.0)
    axis_move_abs("gAxisCmd_X", 0.0)

    # Y 虚拟轴测试（Gantry）
    axis_enable("gAxisCmd_Y")
    axis_move_abs("gAxisCmd_Y", 50.0)
    axis_move_abs("gAxisCmd_Y", 0.0)

except KeyboardInterrupt:
    axis_stop("gAxisCmd_X")
    axis_stop("gAxisCmd_Y")

finally:
    plc.close()
