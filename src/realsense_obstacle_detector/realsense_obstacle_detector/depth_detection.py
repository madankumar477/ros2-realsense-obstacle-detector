"""
Pure-numpy depth-frame obstacle detection for the RealSense D435i blind spot.

No rclpy / ROS message imports here, so everything in this module can be
unit-tested offline with synthetic depth images.

Geometry (camera optical frame: +Z forward, +X right, +Y down; D435i reports
Z-depth, i.e. distance along the optical axis, not radial distance):

  Floor model (per image row v, camera at height h, pitch p, + = tilted up):
      downward_angle(v) = arctan((v - cy) / fy) - p
      floor_Z(v)        = (h - floor_height_margin) / tan(downward_angle(v))
                                                          (inf if angle <= 0)
  A pixel is "above the floor" only when  Z < floor_Z(v) - floor_margin.
  floor_height_margin raises the floor plane, so returns lower than that
  height are ignored. With a low camera the floor is seen at a grazing angle,
  where mm of height error become cm of Z error; a height margin stays
  meaningful at every distance, a Z margin (floor_margin) does not.
  This is one-sided on purpose: returns *behind* the floor plane (multipath,
  reflections) are never treated as obstacles.

  Travel corridor (per image column u):
      X = (u - cx) * Z / fx                 lateral offset in metres
  Only pixels with |X| <= corridor_half_width (robot half-width + margin)
  can trigger, so side walls outside the robot's path are ignored.

  Range band: min_valid_depth <= Z <= detection_distance.  Everything closer
  than min_valid_depth (robot's own front face, flying pixels) is excluded
  from obstacle evidence.

Invalid pixels: RealSense encodes "no depth" as 0; 32FC1 streams may also
contain NaN/inf. Such pixels are never obstacle evidence, but they are also
never evidence that the path is clear: the fraction of *usable* pixels
(finite, > 0 and >= min_valid_depth) is reported so the caller can treat a
blinded/occluded camera as unsafe.
"""

from dataclasses import dataclass
import math

import numpy as np

# Pixels with depth beyond this are physically meaningless for a D435i and are
# treated as invalid data.
MAX_PLAUSIBLE_DEPTH_M = 20.0


@dataclass(frozen=True)
class Intrinsics:
    """Subset of sensor_msgs/CameraInfo needed by the detector."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f'invalid image size {self.width}x{self.height}')
        for name in ('fx', 'fy', 'cx', 'cy'):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite, got {value!r}')
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(
                f'focal lengths must be > 0 (fx={self.fx}, fy={self.fy}); '
                'CameraInfo is probably uncalibrated/empty')


@dataclass(frozen=True)
class DetectionConfig:
    """Geometry and thresholds for a single depth frame."""

    detection_distance: float = 0.70
    min_valid_depth: float = 0.25
    camera_height: float = 0.08
    camera_pitch: float = 0.0
    floor_margin: float = 0.03
    floor_height_margin: float = 0.0
    corridor_half_width: float = 0.45
    roi_top_fraction: float = 0.10
    roi_left_fraction: float = 0.0
    roi_right_fraction: float = 0.0

    def validate(self) -> None:
        if not 0.0 < self.min_valid_depth < self.detection_distance:
            raise ValueError(
                'require 0 < min_valid_depth < detection_distance '
                f'(got {self.min_valid_depth}, {self.detection_distance})')
        if self.detection_distance > MAX_PLAUSIBLE_DEPTH_M:
            raise ValueError(f'detection_distance too large: {self.detection_distance}')
        if self.camera_height <= 0.0:
            raise ValueError(f'camera_height must be > 0, got {self.camera_height}')
        if not -math.pi / 2 < self.camera_pitch < math.pi / 2:
            raise ValueError(f'camera_pitch out of range: {self.camera_pitch}')
        if self.floor_margin < 0.0:
            raise ValueError(f'floor_margin must be >= 0, got {self.floor_margin}')
        if not 0.0 <= self.floor_height_margin < self.camera_height:
            raise ValueError(
                'require 0 <= floor_height_margin < camera_height '
                f'(got {self.floor_height_margin}, {self.camera_height})')
        if self.corridor_half_width <= 0.0:
            raise ValueError(
                f'corridor_half_width must be > 0, got {self.corridor_half_width}')
        for name in ('roi_top_fraction', 'roi_left_fraction', 'roi_right_fraction'):
            value = getattr(self, name)
            if not 0.0 <= value < 1.0:
                raise ValueError(f'{name} must be in [0, 1), got {value}')
        if self.roi_left_fraction + self.roi_right_fraction >= 1.0:
            raise ValueError('roi_left_fraction + roi_right_fraction must be < 1')


@dataclass(frozen=True)
class FrameResult:
    """Outcome of processing one depth frame."""

    obstacle_pixels: int        # pixels inside band, corridor and above floor
    min_distance: float         # closest obstacle Z-depth (m), inf if none
    usable_fraction: float      # usable pixels / ROI pixels, in [0, 1]


def decode_depth(encoding: str, height: int, width: int, step: int,
                 is_bigendian: bool, data, depth_scale: float) -> np.ndarray:
    """Convert raw sensor_msgs/Image fields to a float32 depth array in metres.

    Honours the row stride (`step`) and byte order. Raises ValueError on any
    malformed or unsupported input rather than guessing.
    """
    if encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('>u2' if is_bigendian else '<u2')
        scale = depth_scale
    elif encoding == '32FC1':
        dtype = np.dtype('>f4' if is_bigendian else '<f4')
        scale = 1.0
    else:
        raise ValueError(f'unsupported depth encoding {encoding!r}')
    if height <= 0 or width <= 0:
        raise ValueError(f'invalid image size {width}x{height}')
    if step < width * dtype.itemsize:
        raise ValueError(f'step {step} too small for width {width} ({encoding})')
    # rclpy delivers Image.data as array.array('B'); tests may pass bytes.
    buf = memoryview(data)
    if buf.format != 'B' or buf.ndim != 1:
        buf = buf.cast('B')
    if len(buf) < step * height:
        raise ValueError(
            f'image data truncated: {len(buf)} bytes < step*height {step * height}')
    raw = np.ndarray(shape=(height, width), dtype=dtype, buffer=buf,
                     strides=(step, dtype.itemsize))
    depth = raw.astype(np.float32)
    if scale != 1.0:
        depth *= np.float32(scale)
    return depth


class DepthObstacleDetector:
    """Stateless per-frame detector; geometry tables are built once."""

    def __init__(self, config: DetectionConfig, intrinsics: Intrinsics):
        config.validate()
        intrinsics.validate()
        self.config = config
        self.intrinsics = intrinsics

        h, w = intrinsics.height, intrinsics.width
        self._row0 = int(math.floor(h * config.roi_top_fraction))
        self._col0 = int(math.floor(w * config.roi_left_fraction))
        self._col1 = w - int(math.floor(w * config.roi_right_fraction))
        if self._row0 >= h or self._col0 >= self._col1:
            raise ValueError('ROI is empty for this image size')

        rows = np.arange(h, dtype=np.float64)
        down_angle = np.arctan((rows - intrinsics.cy) / intrinsics.fy) - config.camera_pitch
        with np.errstate(divide='ignore', invalid='ignore'):
            floor_z = np.where(down_angle > 1e-4,
                               (config.camera_height - config.floor_height_margin)
                               / np.tan(down_angle),
                               np.inf)
        # Obstacle threshold per row: Z must be below this to be "above floor".
        self._floor_limit = (floor_z - config.floor_margin)[self._row0:, np.newaxis]
        self.floor_z_per_row = floor_z.astype(np.float32)

        cols = np.arange(w, dtype=np.float64)
        self._lateral_factor = ((cols - intrinsics.cx) / intrinsics.fx)[
            np.newaxis, self._col0:self._col1]

    @property
    def roi_pixel_count(self) -> int:
        return (self.intrinsics.height - self._row0) * (self._col1 - self._col0)

    def process(self, depth_m: np.ndarray) -> FrameResult:
        expected = (self.intrinsics.height, self.intrinsics.width)
        if depth_m.ndim != 2 or depth_m.shape != expected:
            raise ValueError(
                f'depth image shape {depth_m.shape} does not match CameraInfo {expected}')
        cfg = self.config
        roi = depth_m[self._row0:, self._col0:self._col1]

        with np.errstate(invalid='ignore'):
            # 0 = "no return" for RealSense; NaN/inf/absurd values are noise.
            valid = np.isfinite(roi) & (roi > 0.0) & (roi < MAX_PLAUSIBLE_DEPTH_M)
            usable = valid & (roi >= cfg.min_valid_depth)
            in_band = usable & (roi <= cfg.detection_distance)
            above_floor = roi < self._floor_limit
            in_corridor = np.abs(roi * self._lateral_factor) <= cfg.corridor_half_width
            obstacle = in_band & above_floor & in_corridor

        count = int(np.count_nonzero(obstacle))
        min_dist = float(np.min(roi[obstacle])) if count else math.inf
        usable_fraction = float(np.count_nonzero(usable)) / float(roi.size)
        return FrameResult(obstacle_pixels=count, min_distance=min_dist,
                           usable_fraction=usable_fraction)
