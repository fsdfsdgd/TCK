#!/usr/bin/env python3
"""Validate a timestamped TUM trajectory without claiming ground-truth accuracy."""
import argparse
import math

parser = argparse.ArgumentParser()
parser.add_argument("trajectory")
parser.add_argument("--max-speed", type=float, default=None,
                    help="optional dataset-specific numerical-divergence limit in m/s, not an accuracy test")
args = parser.parse_args()
with open(args.trajectory) as source:
    rows = [list(map(float, line.split())) for line in source
            if line.strip() and not line.startswith("#")]
assert len(rows) >= 2, "fewer than two estimated poses"
assert all(len(row) == 8 for row in rows), "expected TUM timestamp xyz xyzw"
assert all(math.isfinite(x) for row in rows for x in row), "NaN/Inf pose"
assert all(b[0] > a[0] for a, b in zip(rows, rows[1:])), "non-increasing timestamps"
assert all(abs(sum(x*x for x in row[4:]) - 1.0) < 1e-5 for row in rows), "invalid quaternion"
steps = [math.sqrt(sum((b[i]-a[i])**2 for i in (1, 2, 3)))
         for a, b in zip(rows, rows[1:])]
speeds = [step/(b[0]-a[0]) for step,a,b in zip(steps,rows,rows[1:])]
if args.max_speed is not None:
    assert math.isfinite(args.max_speed) and args.max_speed > 0, "invalid speed limit"
    assert max(speeds) <= args.max_speed, "trajectory divergence: peak speed {:.3f} > {:.3f} m/s".format(max(speeds),args.max_speed)
print("PASS poses={} span={:.3f}s path_length={:.3f}m max_step={:.3f}m max_speed={:.3f}m/s".format(
    len(rows), rows[-1][0]-rows[0][0], sum(steps), max(steps), max(speeds)))
