"""ROS-free acceptance and best-effort stop helpers for a single live move."""

import math


def displacement_metrics(position, origin, direction):
    values = (*position, *origin, *direction)
    if len(values) != 9 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid position, origin or direction')
    if abs(math.sqrt(sum(v * v for v in direction)) - 1) > 1e-6:
        raise ValueError('Direction must be unit length')
    delta = tuple(a - b for a, b in zip(position, origin))
    projection = sum(a * b for a, b in zip(delta, direction))
    distance = math.sqrt(sum(v * v for v in delta))
    lateral = math.sqrt(max(0, distance * distance - projection * projection))
    return distance, projection, lateral


def require_within_envelope(position, origin, direction, requested_distance):
    if not math.isfinite(requested_distance) or not 0 < requested_distance <= 0.05:
        raise ValueError('Invalid requested envelope')
    distance, projection, lateral = displacement_metrics(position, origin, direction)
    if distance > requested_distance + 0.0001 or lateral > 0.001 or projection < -0.0005:
        raise ValueError('Measured displacement outside requested envelope')
    return distance, projection, lateral


def best_effort_stop(zero_action, stop_action, wait_action):
    """Attempt every stop stage even if a previous stage raises or is refused."""
    result = {'stop_errors': [], 'stop_request_accepted': False,
              'measured_stop_confirmed': False}
    for name, action, key in (
            ('zero publish', zero_action, None),
            ('controller stop', stop_action, 'stop_request_accepted'),
            ('measured stop', wait_action, 'measured_stop_confirmed')):
        try:
            value = action()
            if key:
                result[key] = bool(value)
                if not value:
                    result['stop_errors'].append(name + ' not confirmed')
        except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - finish every stop action
            result['stop_errors'].append(name + ': ' + type(error).__name__)
    return result


def require_target_reached(position, origin, direction, requested_distance, tolerance_m=0.0003):
    if not math.isfinite(tolerance_m) or not 0 < tolerance_m <= 0.0005:
        raise ValueError('Invalid final target tolerance')
    result = require_within_envelope(position, origin, direction, requested_distance)
    if result[1] < requested_distance - tolerance_m:
        raise ValueError('Final measured pose did not reach requested distance')
    return result


def pose_is_after_fence(source_ns, fence_ns):
    return isinstance(source_ns, int) and source_ns > 0 and source_ns > fence_ns
