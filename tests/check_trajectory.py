#!/usr/bin/env python3
"""Validate a timestamped TUM trajectory without claiming ground-truth accuracy."""
import argparse
import math

parser = argparse.ArgumentParser()
parser.add_argument("trajectory")
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
print("PASS poses={} span={:.3f}s path_length={:.3f}m max_step={:.3f}m".format(
    len(rows), rows[-1][0]-rows[0][0], sum(steps), max(steps)))
