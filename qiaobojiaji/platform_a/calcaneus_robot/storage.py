from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path

from .models import PatientCase, Sample


class RecordStore:
    MAX_SAMPLES = 100_000
    HEADER = ["时间", "状态", "X(mm)", "Y(mm)", "Z(mm)", "Rx(deg)", "Ry(deg)", "Rz(deg)", "Fx(N)", "Fy(N)", "Fz(N)", "Tx(Nm)", "Ty(Nm)", "Tz(Nm)", "合力(N)", "进度(%)"]

    def __init__(self, root: Path) -> None:
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)
        self.current_case: PatientCase | None = None
        self.samples: list[Sample] = []
        self.events: list[dict[str, str]] = []

    def begin_case(self, case: PatientCase) -> None:
        self.current_case = case
        self.samples.clear()
        self.events.clear()
        self.event("INFO", f"新建病例：{case.case_id}")

    def event(self, level: str, message: str) -> None:
        self.events.append({"time": datetime.now().isoformat(timespec="seconds"), "level": level, "message": message})

    def sample(self, value: Sample) -> None:
        self.samples.append(value)
        if len(self.samples) > self.MAX_SAMPLES:
            del self.samples[: len(self.samples) - self.MAX_SAMPLES]

    def export(self) -> Path:
        if self.current_case is None:
            raise ValueError("尚未建立病例")
        folder = self.root / self.current_case.case_id
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "case.json").write_text(json.dumps(self.current_case.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        (folder / "events.json").write_text(json.dumps(self.events, ensure_ascii=False, indent=2), encoding="utf-8")
        with (folder / "samples.csv").open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f); writer.writerow(self.HEADER); writer.writerows(s.to_row() for s in self.samples)
        report = folder / "复位过程报告.txt"
        max_force = max((s.wrench.force_norm for s in self.samples), default=0.0)
        report.write_text(
            f"跟骨微创复位辅助机器人过程报告\n病例编号：{self.current_case.case_id}\n患者编码：{self.current_case.patient_code}\n患侧：{self.current_case.side}\n操作者：{self.current_case.operator}\n采样数：{len(self.samples)}\n最大合力：{max_force:.2f} N\n生成时间：{datetime.now():%Y-%m-%d %H:%M:%S}\n声明：本报告由离线演示原型生成，不构成医疗诊断或临床记录。\n",
            encoding="utf-8",
        )
        return folder
