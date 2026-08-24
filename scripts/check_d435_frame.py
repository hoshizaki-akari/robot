#!/usr/bin/env python3
from __future__ import annotations

import struct
import sys
import zlib
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from state_service.d435_monitor import D435Monitor


def decoded_pixels(png: bytes) -> bytes:
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    position = 8
    compressed = b""
    while position < len(png):
        length = struct.unpack(">I", png[position : position + 4])[0]
        kind = png[position + 4 : position + 8]
        data = png[position + 8 : position + 8 + length]
        position += 12 + length
        if kind == b"IDAT":
            compressed += data
        if kind == b"IEND":
            break
    return zlib.decompress(compressed)


def main() -> int:
    rgb_message = SimpleNamespace(
        width=2,
        height=1,
        step=6,
        encoding="rgb8",
        data=bytes([255, 0, 0, 0, 255, 0]),
    )
    assert decoded_pixels(D435Monitor._image_to_png(rgb_message)) == bytes(
        [0, 255, 0, 0, 0, 255, 0]
    )

    bgr_message = SimpleNamespace(
        width=2,
        height=1,
        step=6,
        encoding="bgr8",
        data=bytes([0, 0, 255, 0, 255, 0]),
    )
    assert decoded_pixels(D435Monitor._image_to_png(bgr_message)) == bytes(
        [0, 255, 0, 0, 0, 255, 0]
    )
    print("PASS：D435 的 RGB/BGR 彩色帧都能转换为平台 A 可显示的 PNG")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
