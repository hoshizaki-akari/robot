# aubo_kwr75_sensor

ROS 2 C++ driver for the KWR75D six-axis force/torque sensor. This package is limited to transport and raw, sensor-local measurements. It does not perform force calibration, transform the wrench, or command a robot.

## Verified serial protocol

The vendor protocol used here is RS422 at **460800 baud, 8 data bits, no parity, one stop bit (8N1)**. Each binary sample is exactly 28 bytes:

| Bytes | Meaning |
| --- | --- |
| 0 | `0x48` or `0x49` |
| 1 | `0xAA` |
| 2--25 | six little-endian IEEE-754 `float32`: `Fx`, `Fy`, `Fz` in kgf, then `Mx`, `My`, `Mz` in kgf*m |
| 26--27 | `0x0D 0x0A` |

The decoder accepts arbitrary read boundaries, ignores noise/corrupt frames, and scans forward to recover synchronization. Frames with a non-finite float are rejected. Every measurement is converted internally by `9.80665`, so force is published in N and torque in N*m.

The known stream commands are `48 AA 0D 0A` (start continuous) and `43 AA 0D 0A` (stop). They are never sent by the offline tests. A launched node sends the start command after it has opened and configured the requested device by default; both this behavior and shutdown stop are configurable.

## Run on WSL after device forwarding or local bridge

COM4 is a Windows device. Forward the physical USB serial device to the WSL distribution with `usbipd` first, then identify the resulting Linux device (for example `/dev/ttyUSB0`). Do not pass `COM4` directly as `serial_path`. Confirm device permissions and the actual `/dev/ttyUSB*` mapping before launching. The node has no hardware test mode and does not use replay data as live sensor data.

On this Windows host, `usbipd bind` requires administrator privileges. The tested
alternative is `scripts/kwr75_windows_bridge.ps1` plus
`scripts/kwr75_wsl_pty_bridge.py` from the workstation root. The TCP listener is
bound to `127.0.0.1:18755` only; the WSL relay exposes
`/tmp/kwr75d-bridge-tty`. The existing node then uses that path as `serial_path`
and keeps the same parser, SI conversion, and stale-data health check. A real
28-byte frame and the live ROS wrench topic were verified on 2026-10-15.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run aubo_kwr75_sensor kwr75_sensor_node --ros-args \
  -p serial_path:=/dev/ttyUSB0 \
  -p baud_rate:=460800 \
  -p frame_id:=kwr75_sensor
```

Parameters:

- `serial_path` (default `/dev/ttyUSB0`): forwarded WSL serial device.
- `baud_rate` (default `460800`): only the verified KWR75D baud rate is accepted.
- `frame_id` (default `kwr75_sensor`): sensor-local frame only; no TF transform is applied.
- `stale_timeout_ms` (default `100`): latest valid frame age allowed for health true.
- `status_period_ms` (default `50`): publication period of the health topic.
- `start_stream_on_connect` (default `true`): send the vendor start command after a successful open.
- `stop_stream_on_shutdown` (default `true`): send the vendor stop command during orderly node shutdown.

Topics:

- `/aubo/kwr75/raw_wrench` (`geometry_msgs/msg/WrenchStamped`): only emitted for a valid received frame; header time is the node receipt time and its frame is `frame_id`.
- `/aubo/kwr75/raw_wrench_valid` (`std_msgs/msg/Bool`): true only while the serial session is connected and a valid frame has arrived within `stale_timeout_ms`. Disconnects and stale streams publish false. The driver never manufactures a zero wrench as a valid measurement.

## Offline verification

No serial port is opened by the test binary. The deterministic test suite uses the supplied example frame:

```text
49AABB8A5DBEA17D473FE7D1893D84E8B33C321619BC73DE153B0D0A
```

It checks decoding/SI conversion, fragmented reads, corrupt-frame recovery, non-finite payload rejection, and the two command byte sequences.

```bash
cd /home/zhj/projects/aubo_i5_traction_ws/aubo_i5_traction_workstation/ros2_overlay
source /opt/ros/humble/setup.bash
colcon test --packages-select aubo_kwr75_sensor --event-handlers console_direct+
colcon test-result --all --verbose
```
