#!/usr/bin/env python3
"""Replay saved force inputs through the production AUBO drag model offline."""

import argparse
import csv
import json
import math
import statistics
import subprocess
from itertools import pairwise
from pathlib import Path


def length(vector):
    """Return a vector norm."""
    return math.sqrt(sum(value * value for value in vector))


def analyze(root, session, output):
    """Save the actual C++ model replay and measured-session diagnostics."""
    source = root / "debug" / "traction_sessions" / session / "traction.csv"
    destination = output / session
    destination.mkdir(parents=True, exist_ok=True)
    replay_file = destination / "replay.csv"
    executable = root / "ros2_overlay/build/fr_traction/replay_aubo_drag"
    with replay_file.open("w") as stream:
        subprocess.run([str(executable), str(source)], stdout=stream, check=True)
    with source.open() as stream:
        measured = list(csv.DictReader(stream))
    with replay_file.open() as stream:
        replay = list(csv.DictReader(stream))

    def vector(row, names):
        return [float(row[name]) for name in names]

    q_names = ("ee_qx", "ee_qy", "ee_qz", "ee_qw")
    rotation = None
    if all(name in measured[0] for name in q_names):
        first = vector(measured[0], q_names)
        angles = []
        for row in measured:
            current = vector(row, q_names)
            cosine = abs(sum(a * b for a, b in zip(first, current)))
            cosine /= length(first) * length(current)
            angles.append(math.degrees(2 * math.acos(min(1.0, cosine))))
        rotation = max(angles)

    old = [vector(row, ("old_vx", "old_vy", "old_vz")) for row in replay]
    new = [vector(row, ("new_vx", "new_vy", "new_vz")) for row in replay]

    def roughness(samples, sustained_only=False):
        forces = [length(vector(row, ("fx", "fy", "fz"))) for row in replay]
        differences = [length([a - b for a, b in zip(left, right)])
                       for i, (left, right) in enumerate(pairwise(samples))
                       if not sustained_only or (forces[i] > 0.65 and forces[i + 1] > 0.65)]
        return math.sqrt(statistics.mean(value * value for value in differences))

    opposing = 0
    for row, velocity in zip(replay, new):
        force = vector(row, ("fx", "fy", "fz"))
        opposing += sum(a * b for a, b in zip(force, velocity)) < -1e-12
    result = {
        "session": session,
        "duration_s": float(measured[-1]["elapsed_s"]),
        "actual_final_state": measured[-1]["state"],
        "actual_stop_reason": measured[-1]["stop_reason"],
        "actual_max_orientation_change_deg": rotation,
        "old_command_sample_difference_rms_mps": roughness(old),
        "replayed_command_sample_difference_rms_mps": roughness(new),
        "old_sustained_force_sample_difference_rms_mps": roughness(old, True),
        "replayed_sustained_force_sample_difference_rms_mps": roughness(new, True),
        "replayed_opposing_force_command_rows": opposing,
        "interpretation": (
            "Input replay only. Measured pose belongs to the old controller; "
            "this does not predict or validate new robot motion. "
            "Replay assumes identity drag signs/axis mask as base-force input."
        ),
    }
    (destination / "analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    """Review named local sessions without hardware or a web connection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    output = root / "debug/session_20261021_aubo_drag_v2"
    for session in args.sessions:
        print(json.dumps(analyze(root, session, output), ensure_ascii=False))


if __name__ == "__main__":
    main()
