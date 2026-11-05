# Test cases updated for source-workspace camera geometry by E MADAN KUMAR (2026).
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

from realsense_obstacle_detector.depth_detection import (  # noqa: E402
    DepthObstacleDetector, DetectionConfig, Intrinsics, decode_depth)
from synthetic import D435I, add_box, render_floor, to_16uc1_bytes  # noqa: E402

CFG = DetectionConfig(
    detection_distance=0.40, min_valid_depth=0.25, camera_height=0.077,
    camera_pitch=-0.0274, floor_margin=0.03, floor_height_margin=0.025,
    corridor_half_width=0.45, roi_top_fraction=0.10)  # source-workspace camera settings
THRESHOLD = 20            # obstacle_pixel_threshold default


@pytest.fixture
def det():
    return DepthObstacleDetector(CFG, D435I)


def test_empty_floor_is_clear(det):
    r = det.process(render_floor())
    assert r.obstacle_pixels == 0
    assert math.isinf(r.min_distance)
    assert r.usable_fraction > 0.5


def test_noisy_floor_is_clear(det):
    # 5 mm (1 sigma) depth noise, typical for a D435i at < 1 m.
    r = det.process(render_floor(noise=0.005, seed=3))
    assert r.obstacle_pixels < THRESHOLD


def test_box_in_path_is_detected(det):
    r = det.process(add_box(render_floor(height=CFG.camera_height, pitch=CFG.camera_pitch), z=0.35, x_left=-0.15, x_right=0.15, top=0.30, cam_height=CFG.camera_height))
    assert r.obstacle_pixels >= THRESHOLD
    assert r.min_distance == pytest.approx(0.35, abs=0.01)


@pytest.mark.parametrize('z', [0.30, 0.34, 0.38, 0.39])
def test_box_across_blind_band_is_detected(det, z):
    r = det.process(add_box(render_floor(height=CFG.camera_height, pitch=CFG.camera_pitch), z=z, x_left=-0.10, x_right=0.10, top=0.20, cam_height=CFG.camera_height))
    assert r.obstacle_pixels >= THRESHOLD


def test_low_obstacle_4cm_detected(det):
    r = det.process(add_box(render_floor(height=CFG.camera_height, pitch=CFG.camera_pitch), z=0.35, x_left=-0.10, x_right=0.10, top=0.04, cam_height=CFG.camera_height))
    assert r.obstacle_pixels >= THRESHOLD


def test_box_beyond_band_ignored(det):
    # Beyond the configured camera detection band: ignore it.
    r = det.process(add_box(render_floor(), z=1.00, x_left=-0.3, x_right=0.3, top=0.40))
    assert r.obstacle_pixels == 0


def test_box_outside_corridor_ignored(det):
    # Wall-like object 0.6..1.0 m to the side, inside the depth band.
    r = det.process(add_box(render_floor(height=CFG.camera_height, pitch=CFG.camera_pitch), z=0.35, x_left=0.60, x_right=1.0, top=0.40, cam_height=CFG.camera_height))
    assert r.obstacle_pixels == 0


def test_box_at_corridor_edge_detected(det):
    # Robot half-width is 0.40 m: an object at X 0.30..0.42 must stop us.
    r = det.process(add_box(render_floor(height=CFG.camera_height, pitch=CFG.camera_pitch), z=0.38, x_left=0.30, x_right=0.42, top=0.30, cam_height=CFG.camera_height))
    assert r.obstacle_pixels >= THRESHOLD


def test_returns_behind_floor_are_not_obstacles(det):
    depth = render_floor()
    # Multipath/reflection: floor pixels reading deeper than the floor plane.
    rows = slice(300, 360)
    depth[rows, 300:500] = np.where(depth[rows, 300:500] > 0, depth[rows, 300:500] + 0.2, 0)
    assert det.process(depth).obstacle_pixels == 0


def test_all_zero_frame_is_not_clear(det):
    r = det.process(np.zeros((480, 848), np.float32))
    assert r.obstacle_pixels == 0
    assert r.usable_fraction == 0.0      # the latch turns this into BLIND


def test_nan_frame_is_not_clear(det):
    r = det.process(np.full((480, 848), np.nan, np.float32))
    assert r.usable_fraction == 0.0


def test_object_inside_sensor_minimum_range_blinds_camera(det):
    # Object 0.15 m from the lens filling the view: sensor returns zeros.
    depth = add_box(render_floor(), z=0.15, x_left=-0.5, x_right=0.5, top=0.5)
    r = det.process(depth)
    assert r.usable_fraction < 0.2


def test_robot_face_band_excluded(det):
    # Returns at 0.18-0.24 m (robot face / flying pixels) never trigger.
    depth = render_floor()
    depth[200:260, 380:470] = 0.20
    assert det.process(depth).obstacle_pixels == 0


def test_shape_mismatch_raises(det):
    with pytest.raises(ValueError):
        det.process(np.zeros((240, 424), np.float32))


def test_overestimated_camera_height_fails_towards_blocking():
    # Documents why camera_height must be measured: configuring 0.10 m on a
    # camera that is really at 0.08 m makes the floor look like an obstacle
    # (fail-safe direction, but the robot would not drive forward).
    det = DepthObstacleDetector(DetectionConfig(camera_height=0.10), D435I)
    assert det.process(render_floor(height=0.08)).obstacle_pixels >= THRESHOLD


def test_pitched_camera_floor_model():
    pitch = math.radians(-5.0)   # tilted 5 deg down
    det = DepthObstacleDetector(DetectionConfig(camera_pitch=pitch), D435I)
    assert det.process(render_floor(pitch=pitch)).obstacle_pixels == 0


def test_floor_height_margin_absorbs_camera_settling():
    # Camera sits 7 mm lower and 0.4 deg further down than configured: with
    # only the Z margin the grazing floor triggers, the height margin clears it.
    actual = render_floor(height=0.073, pitch=math.radians(-1.8))
    base = dict(camera_height=0.08, camera_pitch=math.radians(-1.4))
    assert DepthObstacleDetector(DetectionConfig(**base), D435I).process(
        actual).obstacle_pixels > 20
    det = DepthObstacleDetector(DetectionConfig(floor_height_margin=0.025, **base), D435I)
    assert det.process(actual).obstacle_pixels == 0


def test_floor_height_margin_still_detects_low_obstacle():
    cfg = DetectionConfig(floor_height_margin=0.025)
    depth = add_box(render_floor(), z=0.35, x_left=-0.1, x_right=0.1, top=0.04)
    assert DepthObstacleDetector(cfg, D435I).process(depth).obstacle_pixels > 20


@pytest.mark.parametrize('kwargs', [
    dict(min_valid_depth=0.8),             # band inverted
    dict(floor_height_margin=-0.01),
    dict(floor_height_margin=0.08),        # not below camera_height
    dict(camera_height=0.0),
    dict(corridor_half_width=0.0),
    dict(roi_left_fraction=0.6, roi_right_fraction=0.5),
    dict(floor_margin=-0.01),
])
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        DepthObstacleDetector(DetectionConfig(**kwargs), D435I)


def test_invalid_intrinsics_rejected():
    with pytest.raises(ValueError):
        DepthObstacleDetector(CFG, Intrinsics(848, 480, 0.0, 0.0, 424.0, 240.0))


# ── decode_depth ────────────────────────────────────────────────────────────

def test_decode_16uc1_with_row_padding_and_endianness():
    depth = add_box(render_floor(), z=0.5, x_left=-0.1, x_right=0.1, top=0.2)
    for big in (False, True):
        data, step = to_16uc1_bytes(depth, step_pad=16, big_endian=big)
        out = decode_depth('16UC1', 480, 848, step, big, data, 0.001)
        np.testing.assert_allclose(out, depth, atol=0.0006)


def test_decode_32fc1():
    depth = render_floor()
    out = decode_depth('32FC1', 480, 848, 848 * 4, False, depth.astype('<f4').tobytes(), 0.001)
    np.testing.assert_array_equal(out, depth)


@pytest.mark.parametrize('args', [
    ('rgb8', 480, 848, 848 * 3, False, bytes(480 * 848 * 3)),
    ('16UC1', 480, 848, 848 * 2, False, bytes(100)),            # truncated
    ('16UC1', 480, 848, 100, False, bytes(480 * 848 * 2)),      # step too small
])
def test_decode_rejects_bad_input(args):
    with pytest.raises(ValueError):
        decode_depth(*args, 0.001)
