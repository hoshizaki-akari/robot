from __future__ import annotations

import glob
from pathlib import Path

import serial


PREFERRED_DEVICE = (
    "/dev/serial/by-id/"
    "usb-FTDI_FT232R_USB_UART_A10KATOF-if00-port0"
)
REG_INIT_STATUS = 0x0200
REG_GRIP_STATUS = 0x0201
REG_ACTUAL_POSITION = 0x0202


def find_device() -> str:
    if Path(PREFERRED_DEVICE).exists():
        return PREFERRED_DEVICE
    matches = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
    if len(matches) == 1:
        return matches[0]
    matches = sorted(glob.glob("/dev/ttyUSB*"))
    if len(matches) == 1:
        return matches[0]
    raise FileNotFoundError("WSL 中没有找到唯一的 AG95 FTDI 串口")


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def read_register(port: serial.Serial, register: int) -> int:
    payload = bytes((1, 3, register >> 8, register & 0xFF, 0, 1))
    request = payload + crc16(payload)
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    response = port.read(7)
    if len(response) != 7:
        raise TimeoutError("AG95 响应超时")
    if crc16(response[:-2]) != response[-2:]:
        raise ValueError("AG95 响应 CRC 错误")
    if response[:3] != bytes((1, 3, 2)):
        raise ValueError(f"AG95 响应格式异常：{response.hex(' ')}")
    return (response[3] << 8) | response[4]


def read_status() -> dict[str, int | str]:
    device = find_device()
    with serial.Serial(
        device,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.35,
        write_timeout=0.35,
        exclusive=True,
    ) as port:
        return {
            "device": device,
            "init_status": read_register(port, REG_INIT_STATUS),
            "motion_status": read_register(port, REG_GRIP_STATUS),
            "position_raw": read_register(port, REG_ACTUAL_POSITION),
        }

