from dynamixel_sdk import *
import time

try:
    from . import config
except ImportError:
    import config

DEVICENAME = getattr(config, "DEVICENAME", "/dev/ttyUSB0")
BAUDRATE = getattr(config, "BAUDRATE", 57600)
PROTOCOL_VERSION = config.DYNAMIXEL_PROTOCOL_VERSION
DXL_IDS = getattr(config, "MOTOR_NUM", [0, 1])

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    raise RuntimeError("Failed to open port")

if not portHandler.setBaudRate(BAUDRATE):
    raise RuntimeError("Failed to set baudrate")

for dxl_id in DXL_IDS:
    print(f"\nRebooting DYNAMIXEL ID {dxl_id}...")

    comm_result, dxl_error = packetHandler.reboot(portHandler, dxl_id)

    if comm_result != COMM_SUCCESS:
        print("COMM ERROR:", packetHandler.getTxRxResult(comm_result))
    else:
        print("Reboot command sent.")

    if dxl_error != 0:
        print("DXL ERROR:", dxl_error)

    time.sleep(1.0)

    # reboot 후 상태 확인
    model, comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
    print("Ping model:", model)
    print("comm:", comm_result)
    print("err:", dxl_error)

    # Hardware Error Status 확인
    hw_error, comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, dxl_id, 70
    )
    print("Hardware Error Status:", hw_error)
    print("comm:", comm_result)
    print("err:", dxl_error)

portHandler.closePort()
