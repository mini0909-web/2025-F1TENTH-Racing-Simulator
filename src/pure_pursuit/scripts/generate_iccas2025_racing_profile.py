#!/usr/bin/env python3
"""
Build an offline racing-line profile (position, curvature, speed, steer, boundaries)
for the ICCAS2025 map using the provided spline path CSV.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Tuple


ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = ROOT / "racecar_simulator" / "maps" / "f1tenth_racetracks" / "iccas2025"
INPUT = MAP_DIR / "iccas2025_path_spline.csv"
OUTPUT = MAP_DIR / "iccas2025_racing_profile.csv"

# Vehicle / track assumptions (aligned to pure_pursuit defaults)
WHEELBASE = 0.34
MU = 1.0  # friction coeff
G = 9.81
LAT_ACC_MAX = MU * G
V_MAX = 7.0
V_MIN = 1.0
A_ACCEL = 3.5   # forward accel limit
A_DECEL = 3.5   # braking limit (positive magnitude)


def read_rows(path: Path) -> List[Tuple[float, float, float, float]]:
  raw = path.read_text().splitlines()
  if not raw:
    raise RuntimeError(f"{path} is empty")
  # clean header (strip leading '#', spaces)
  header = raw[0]
  if header.startswith("#"):
    header = header.lstrip("#").strip()
  cleaned = "\n".join([header] + raw[1:])
  reader = csv.DictReader(cleaned.splitlines(), skipinitialspace=True)
  rows = []
  for row in reader:
    x = float(row["x_m"])
    y = float(row["y_m"])
    wl = float(row["w_tr_left_m"])
    wr = float(row["w_tr_right_m"])
    rows.append((x, y, wl, wr))
  return rows


def yaw_from_pts(xs: List[float], ys: List[float]) -> List[float]:
  n = len(xs)
  out = []
  for i in range(n):
    ip = (i - 1) % n
    inx = (i + 1) % n
    dx = xs[inx] - xs[ip]
    dy = ys[inx] - ys[ip]
    out.append(math.atan2(dy, dx))
  return out


def curvature(xs: List[float], ys: List[float]) -> List[float]:
  n = len(xs)
  k_list = []
  for i in range(n):
    ip = (i - 1) % n
    inx = (i + 1) % n
    ax, ay = xs[ip], ys[ip]
    bx, by = xs[i], ys[i]
    cx, cy = xs[inx], ys[inx]
    ab = math.hypot(bx - ax, by - ay)
    bc = math.hypot(cx - bx, cy - by)
    ca = math.hypot(ax - cx, ay - cy)
    denom = ab * bc * ca
    if denom < 1e-6:
      k_list.append(0.0)
      continue
    area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    k_list.append(2.0 * area2 / denom)
  return k_list


def cumulative_s(xs: List[float], ys: List[float]) -> Tuple[List[float], List[float]]:
  n = len(xs)
  ds_segments: List[float] = []
  for i in range(n):
    ib = (i + 1) % n
    ds_segments.append(math.hypot(xs[ib] - xs[i], ys[ib] - ys[i]))
  s = [0.0]
  for i in range(1, n):
    s.append(s[-1] + ds_segments[i - 1])
  return s, ds_segments


def speed_profile(k_list: List[float], ds_segments: List[float]) -> List[float]:
  n = len(k_list)
  v = [0.0] * n
  for i in range(n):
    kappa = abs(k_list[i])
    v_curve = math.sqrt(max(LAT_ACC_MAX / max(kappa, 1e-3), 0.0))
    v[i] = max(V_MIN, min(V_MAX, v_curve))

  # forward accel pass
  for i in range(1, n):
    ds = ds_segments[i - 1]
    v[i] = min(v[i], math.sqrt(v[i - 1] * v[i - 1] + 2 * A_ACCEL * ds))

  # backward braking pass (wrap closed loop)
  for i in range(n - 2, -1, -1):
    ds = ds_segments[i]
    v[i] = min(v[i], math.sqrt(v[i + 1] * v[i + 1] + 2 * A_DECEL * ds))
  v0_from_last = math.sqrt(v[-1] * v[-1] + 2 * A_DECEL * ds_segments[-1])
  v[0] = min(v[0], v0_from_last)
  return v


def main() -> None:
  rows = read_rows(INPUT)
  xs = [r[0] for r in rows]
  ys = [r[1] for r in rows]
  wl = [r[2] for r in rows]
  wr = [r[3] for r in rows]

  yaw = yaw_from_pts(xs, ys)
  k_list = curvature(xs, ys)
  s, ds_segments = cumulative_s(xs, ys)
  v = speed_profile(k_list, ds_segments)

  normals = [(-math.sin(yaw[i]), math.cos(yaw[i])) for i in range(len(xs))]
  left = [(xs[i] + wl[i] * normals[i][0], ys[i] + wl[i] * normals[i][1]) for i in range(len(xs))]
  right = [(xs[i] - wr[i] * normals[i][0], ys[i] - wr[i] * normals[i][1]) for i in range(len(xs))]
  steer = [max(-0.42, min(0.42, WHEELBASE * k_list[i])) for i in range(len(xs))]

  OUTPUT.parent.mkdir(parents=True, exist_ok=True)
  with OUTPUT.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
      "x_m", "y_m", "yaw_rad", "curvature", "speed_mps", "steer_rad",
      "left_x_m", "left_y_m", "right_x_m", "right_y_m",
      "w_tr_left_m", "w_tr_right_m", "s_m"
    ])
    for i in range(len(xs)):
      writer.writerow([
        xs[i], ys[i], yaw[i], k_list[i], v[i], steer[i],
        left[i][0], left[i][1], right[i][0], right[i][1],
        wl[i], wr[i], s[i]
      ])
  print(f"Wrote {OUTPUT} with {len(xs)} points")


if __name__ == "__main__":
  main()