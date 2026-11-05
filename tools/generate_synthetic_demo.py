#!/usr/bin/env python3
"""Generate a reproducible synthetic depth/mask figure from package test helpers."""
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src' / 'realsense_obstacle_detector'
sys.path.insert(0, str(PKG / 'test'))
sys.path.insert(0, str(PKG))

from synthetic import D435I, add_box, render_floor  # noqa: E402
from realsense_obstacle_detector.depth_detection import (  # noqa: E402
    DepthObstacleDetector, DetectionConfig,
)

cfg = DetectionConfig(
    detection_distance=0.40,
    min_valid_depth=0.25,
    camera_height=0.077,
    camera_pitch=-0.0274,
    floor_margin=0.03,
    floor_height_margin=0.025,
    corridor_half_width=0.45,
    roi_top_fraction=0.10,
    roi_left_fraction=0.0,
    roi_right_fraction=0.0,
)
intr = D435I
base = render_floor(intr=intr, height=cfg.camera_height, pitch=cfg.camera_pitch, noise=0.001, seed=17)
depth = add_box(base, z=0.35, x_left=-0.15, x_right=0.15, top=0.30, cam_height=cfg.camera_height, intr=intr)
detector = DepthObstacleDetector(cfg, intr)
result = detector.process(depth)

# Recreate the exact boolean predicate used by DepthObstacleDetector.process,
# using its already-computed row and column geometry arrays.
roi = depth[detector._row0:, detector._col0:detector._col1]
valid = np.isfinite(roi) & (roi > 0.0) & (roi < 20.0)
usable = valid & (roi >= cfg.min_valid_depth)
in_band = usable & (roi <= cfg.detection_distance)
above_floor = roi < detector._floor_limit
in_corridor = np.abs(roi * detector._lateral_factor) <= cfg.corridor_half_width
mask_roi = in_band & above_floor & in_corridor
mask = np.zeros(depth.shape, dtype=np.uint8)
mask[detector._row0:, detector._col0:detector._col1] = mask_roi.astype(np.uint8) * 255

# Depth preview: normalize metres into a Turbo heat map; missing returns black.
vis_depth = np.clip(depth, 0.0, 2.0)
normalized = (vis_depth / 2.0 * 255.0).astype(np.uint8)
depth_color = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
depth_color[depth <= 0.0] = (0, 0, 0)
depth_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)
mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

W, H = 1600, 780
canvas = Image.new('RGB', (W, H), (13, 19, 31))
d = ImageDraw.Draw(canvas)
font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font_bold_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def font(size, bold=False):
    path = font_bold_path if bold else font_path
    return ImageFont.truetype(path, size)

# Header
d.text((55, 36), 'ROS 2 REALSENSE OBSTACLE DETECTOR', font=font(38, True), fill=(240, 246, 255))
d.text((58, 92), 'Synthetic camera-geometry demo • source-workspace configuration', font=font(22), fill=(150, 190, 230))
d.rounded_rectangle((55, 145, 1545, 215), radius=18, fill=(25, 38, 58), outline=(66, 117, 183), width=2)
d.text((82, 164), 'SIMULATED DEPTH FRAME — NOT A PHYSICAL CAMERA CAPTURE', font=font(24, True), fill=(255, 208, 112))

# Panels
panels = [(55, 255, 535, 660), (560, 255, 1040, 660), (1065, 255, 1545, 660)]
labels = ['Synthetic depth image (metres)', 'Detector mask (ROI + floor + corridor)', 'Result from package detector']
for box, label in zip(panels, labels):
    d.rounded_rectangle(box, radius=18, fill=(21, 31, 48), outline=(72, 113, 168), width=2)
    d.text((box[0] + 20, box[1] + 16), label, font=font(21, True), fill=(225, 237, 255))

# Fit previews into panel bodies.
def fit_rgb(arr, max_size):
    im = Image.fromarray(arr)
    im.thumbnail(max_size, Image.Resampling.LANCZOS)
    return im

preview_size = (440, 315)
for arr, box in zip([depth_rgb, mask_rgb], panels[:2]):
    im = fit_rgb(arr, preview_size)
    x = box[0] + (box[2]-box[0]-im.width)//2
    y = box[1] + 70 + (300-im.height)//2
    canvas.paste(im, (x, y))

# Result card
box = panels[2]
d.text((box[0]+28, box[1]+95), 'Obstacle pixels', font=font(20), fill=(161, 190, 225))
d.text((box[0]+28, box[1]+128), f'{result.obstacle_pixels:,}', font=font(38, True), fill=(112, 230, 170))
d.text((box[0]+28, box[1]+206), 'Minimum depth', font=font(20), fill=(161, 190, 225))
d.text((box[0]+28, box[1]+239), f'{result.min_distance:.3f} m', font=font(38, True), fill=(112, 230, 170))
d.text((box[0]+28, box[1]+317), 'Detector decision', font=font(20), fill=(161, 190, 225))
d.rounded_rectangle((box[0]+26, box[1]+350, box[2]-26, box[1]+390), radius=10, fill=(29, 88, 63))
d.text((box[0]+48, box[1]+358), 'OBSTACLE IN DETECTION BAND', font=font(18, True), fill=(226, 255, 236))

# Footer
d.text((58, 706), 'Generated with test/synthetic.py and DepthObstacleDetector; no live D435i data used.', font=font(18), fill=(175, 192, 215))
d.text((58, 738), 'Configuration: 0.25–0.40 m valid band • camera height 0.077 m • pitch −0.0274 rad', font=font(16), fill=(135, 160, 193))

out = ROOT / 'assets' / 'synthetic_depth_detection.png'
canvas.save(out, optimize=True)
print(f'Wrote {out}')
print(f'obstacle_pixels={result.obstacle_pixels}, min_distance={result.min_distance:.4f}m, usable_fraction={result.usable_fraction:.3f}')
