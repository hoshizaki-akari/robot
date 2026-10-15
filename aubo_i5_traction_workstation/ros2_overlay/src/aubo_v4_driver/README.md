# AUBOPE V4 state adapter

`aubo_v4_state_node` is the sole owner of one legacy AUBOPE V4 `ServiceInterface` session. It is limited to read-only state polling and an operator-requested hardware stop. It has no motion, power, brake-release, fault-clear, force-control, or I/O command endpoint.

- Real V4 `wayPoint_S` data is published as `/joint_states` and `/controller_manager/ee_state` (`fr_traction/msg/EndEffectorState`).
- Linear twist is a finite-difference estimate only after two valid polls; angular twist remains zero because no V4 twist source is verified.
- `/controller_manager/healthy` is true only after a successful real state read and a V4 diagnosis with arm power on and no checked faults. A TCP login alone is never healthy.
- `/diagnostics` carries explicit state/failure reasons. There are no unverified motion services or command subscriptions.
- `/aubo_i5/driver_identity` is transient-local and republished every second with the exact value `AUBO_I5_V4`, including while login is disabled, so integrations can reject an accidental FR5 ROS graph.
- `~/hardware_stop` is `std_srvs/srv/Trigger` and invokes only `robotMoveFastStop()`. Its SDK result is checked. A returned success latches health false; it does not claim completed stopping. Operator review and a node restart are required before health can become true again.

`pose_frame_id` labels the V4 waypoint pose. Set it only to the AUBO-specific calibrated frame the controller actually reports; the default `aubo_base` is a label, not a calibration or TF assertion.

## Credentials and connection

All connection inputs are read from the process environment and are never logged:

```bash
export AUBO_V4_SDK_ROOT=/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit
export AUBO_V4_HOST='controller-host-or-ip'
export AUBO_V4_PORT=8899
export AUBO_V4_USERNAME='operator-name'
export AUBO_V4_PASSWORD='secret'
```

Missing environment values, login errors, and state-read errors publish unhealthy diagnostics. The driver does not automatically reconnect, power on, release brakes, or clear faults. Do not start this node during offline validation; tests never construct it or open a controller session.

## Offline build and tests

From `aubo_i5_traction_workstation/ros2_overlay`, source the Humble underlay and existing overlay containing `fr_traction`, then run:

```bash
export AUBO_V4_SDK_ROOT=/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit
colcon build --packages-select aubo_v4_driver
colcon test --packages-select aubo_v4_driver
colcon test-result --verbose
```

The SDK remains external. CMake requires `include/aubo_driver/serviceinterface.h` and `lib/lib64/aubocontroller/libauborobotcontroller.so` below `AUBO_V4_SDK_ROOT`; no vendor binary is copied into the repository.

The supplied audit bundle's controller library also requires the legacy SONAME `libprotobuf.so.9`, which is not included in that bundle and is not ABI-compatible with Humble's system protobuf. The offline build therefore validates source compilation and links with unresolved shared-library dependencies allowed; do not start the node until the matching vendor runtime has been installed outside the repository.
