# AUBO i5 Description

This ROS 2 `ament_cmake` package provides a visualization-only description of
the AUBO i5 V4-era arm. It installs one URDF and the visual and collision mesh
assets that the URDF references.

## Scope and safety boundary

This package contains no driver, controller, launch file, MoveIt configuration,
motion-planning configuration, simulation, or robot connection code. Building
or loading this model does not validate a physical robot and must not be used to
command one.

The supplied joint transforms are a mechanically ported vendor model for
visualization. AUBO-specific DH/kinematic calibration, base frame, TCP/tool
frame, joint-axis signs, payload, and motion limits remain uncalibrated in this
project. Do not use this description for MoveIt or motion planning until those
items have been measured and approved at a real-hardware checkpoint.

The source URDF declares zero effort and velocity limits. They are deliberately
preserved because no display-parser requirement demonstrates that they must be
changed, and this package must not invent movement limits.

## Build

From the ROS 2 overlay workspace after sourcing its ROS distribution:

```bash
colcon build --packages-select aubo_i5_description
```

For source lineage and the exact mechanical extraction, see
[`PROVENANCE.md`](PROVENANCE.md). The copied upstream BSD-3-Clause text is in
[`LICENSE`](LICENSE).
