from dynamixel_sdk import *

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
ADDR_TORQUE_ENABLE = 64

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    raise RuntimeError("Failed to open port")

if not portHandler.setBaudRate(BAUDRATE):
    raise RuntimeError("Failed to set baudrate")

def read1(dxl_id, addr):
    value, comm, err = packetHandler.read1ByteTxRx(portHandler, dxl_id, addr)
    return value, comm, err

def read2(dxl_id, addr):
    value, comm, err = packetHandler.read2ByteTxRx(portHandler, dxl_id, addr)
    return value, comm, err

for dxl_id in [0, 1]:
    model, comm, err = packetHandler.ping(portHandler, dxl_id)

    print(f"\nID {dxl_id}")
    print("ping:", model, "comm:", comm, "err:", err, "alert:", bool(err & 0x80))

    hw, comm, err = read1(dxl_id, ADDR_HARDWARE_ERROR_STATUS)
    print("Hardware Error Status(70):", hw, "comm:", comm, "err:", err)

    volt, comm, err = read2(dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
    print("Present Input Voltage(144):", volt / 10.0, "V", "comm:", comm, "err:", err)

    temp, comm, err = read1(dxl_id, ADDR_PRESENT_TEMPERATURE)
    print("Present Temperature(146):", temp, "C", "comm:", comm, "err:", err)

    torque, comm, err = read1(dxl_id, ADDR_TORQUE_ENABLE)
    print("Torque Enable(64):", torque, "comm:", comm, "err:", err)

portHandler.closePort()