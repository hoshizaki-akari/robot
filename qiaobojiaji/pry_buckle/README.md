# 撬拨：水平直径夹持点（独立模块）

本目录是独立于 `platform_a` 的新实现；不会改动 `platform_a`、`state_service`、或任何真机运动脚本。

规则固定如下：

1. 复用 `platform_a/models/heel_seg.pt` 得到足跟掩码；
2. 仅取掩码质心所在的图像水平线；
3. 在该水平线上求足跟左/右边界，作为夹持直径两端；
4. 仅从两个端点内侧的局部足跟深度计算三维点；
5. 仅当真实三维宽度处于 50–60 mm 时结果才有效。超出范围会明确拒绝，绝不套用人工偏移或把数值强制改成目标值。

离线检查（已保存的对齐深度 `.npy`）：

```bash
source .venv/bin/activate
PYTHONPATH=. python scripts/pry_horizontal_diameter.py \
  --color COLOR.png --depth-npy DEPTH.npy --camera-info CAMERA_INFO.json
```

输出为 `debug/pry_buckle/horizontal_diameter_overlay.png` 与 JSON 结果。
