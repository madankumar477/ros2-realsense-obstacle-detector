"""Synthetic D435i depth frames rendered from the camera geometry (offline tests)."""

import numpy as np

from realsense_obstacle_detector.depth_detection import Intrinsics

# Typical D435i depth intrinsics at the driver's default 848x480 profile.
D435I = Intrinsics(width=848, height=480, fx=425.0, fy=425.0, cx=424.0, cy=240.0)
SENSOR_MIN_Z = 0.19   # D435i returns 0 (no data) closer than ~0.19 m at 848x480
WALL_Z = 5.0


def render_floor(intr=D435I, height=0.08, pitch=0.0, noise=0.0, seed=0):
    """Empty scene: flat floor below the horizon, wall at WALL_Z above it."""
    rows = np.arange(intr.height, dtype=np.float64)
    down = np.arctan((rows - intr.cy) / intr.fy) - pitch
    with np.errstate(divide='ignore'):
        floor_z = np.where(down > 1e-4, height / np.tan(down), np.inf)
    z = np.minimum(floor_z, WALL_Z)[:, None] * np.ones((1, intr.width))
    if noise:
        z = z + np.random.default_rng(seed).normal(0.0, noise, z.shape)
    z[z < SENSOR_MIN_Z] = 0.0
    return z.astype(np.float32)


def add_box(depth, z, x_left, x_right, top, bottom=0.0, cam_height=0.08, intr=D435I):
    """Fronto-parallel box face at depth z spanning lateral [x_left, x_right]
    and heights [bottom, top] above the floor (metres)."""
    out = depth.copy()
    u0 = int(np.ceil(intr.cx + intr.fx * x_left / z))
    u1 = int(np.floor(intr.cx + intr.fx * x_right / z))
    v0 = int(np.ceil(intr.cy + intr.fy * (cam_height - top) / z))
    v1 = int(np.floor(intr.cy + intr.fy * (cam_height - bottom) / z))
    u0, u1 = max(u0, 0), min(u1, intr.width - 1)
    v0, v1 = max(v0, 0), min(v1, intr.height - 1)
    if u0 <= u1 and v0 <= v1:
        region = out[v0:v1 + 1, u0:u1 + 1]
        # Box occludes only what lies behind it.
        mask = (region == 0.0) | (region > z)
        region[mask] = 0.0 if z < SENSOR_MIN_Z else z
    return out


def to_16uc1_bytes(depth_m, step_pad=0, big_endian=False):
    mm = np.round(depth_m * 1000.0).astype('>u2' if big_endian else '<u2')
    if step_pad:
        padded = np.zeros((mm.shape[0], mm.shape[1] + step_pad), dtype=mm.dtype)
        padded[:, :mm.shape[1]] = mm
        mm = padded
    return mm.tobytes(), mm.shape[1] * 2
