#!/usr/bin/env python3
"""只看 D435 彩色画面，不识别、不控制机械臂、不控制夹爪。"""

from __future__ import annotations

import base64
import tkinter as tk
from urllib.error import URLError
from urllib.request import urlopen


IMAGE_URL = "http://127.0.0.1:8765/api/d435/color.png"
REFRESH_MS = 100


class CameraWindow:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("D435 摄像头画面（只看，不控制机器人）")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.image_label = tk.Label(self.root, text="正在等待摄像头画面…", bg="#202124")
        self.image_label.pack(padx=8, pady=8)
        self.status = tk.Label(self.root, text="", anchor="w")
        self.status.pack(fill="x", padx=8, pady=(0, 8))
        self.photo: tk.PhotoImage | None = None
        self.closed = False

    def update(self) -> None:
        if self.closed:
            return
        try:
            with urlopen(IMAGE_URL, timeout=1.5) as response:
                png = response.read()
            encoded = base64.b64encode(png).decode("ascii")
            self.photo = tk.PhotoImage(data=encoded)
            self.image_label.configure(image=self.photo, text="")
            self.status.configure(text="摄像头正常。关闭窗口即可结束。")
        except (OSError, URLError, tk.TclError) as exc:
            self.status.configure(text=f"暂时没有画面：{exc}")
        self.root.after(REFRESH_MS, self.update)

    def close(self) -> None:
        self.closed = True
        self.root.destroy()

    def run(self) -> None:
        self.update()
        self.root.mainloop()


if __name__ == "__main__":
    CameraWindow().run()
