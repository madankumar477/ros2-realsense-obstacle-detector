# Standalone project documentation/generalization updated by E MADAN KUMAR (2026).
#!/usr/bin/env python3
"""
RealSense D435i blind-spot obstacle detector (fail-safe).

Covers a configurable close-range zone in front of a mobile robot. This node
reports whether the configured safety controller should inhibit forward motion.

Published (consumed by cmd_vel_safety_gate, which clamps linear.x <= 0
while True; rotation and reverse still pass so Nav2 can turn away):
  /realsense/obstacle_detected      std_msgs/Bool     (heartbeat at publish_rate)
  /realsense/min_obstacle_distance  std_msgs/Float32  (-1.0 when no obstacle)
  /realsense/obstacle_alert         std_msgs/String   (reason, only while blocked)

Subscribed:
  <depth_image_topic>   sensor_msgs/Image       16UC1 (mm) or 32FC1 (m)
  <camera_info_topic>   sensor_msgs/CameraInfo  (default: derived from depth topic)

Fail-safe: /realsense/obstacle_detected is True (forward motion inhibited)
until valid frames are flowing, whenever frames go stale, CameraInfo is
missing, a frame cannot be processed, or the image is mostly invalid.
See safety_state.py for the exact latch policy and depth_detection.py for
the geometry.

Live tuning (validated; rejected if inconsistent), e.g.:
  ros2 param set /realsense_obstacle_detector camera_height 0.082
  ros2 param set /realsense_obstacle_detector floor_margin 0.03
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
import tf2_ros

from .depth_detection import (DepthObstacleDetector, DetectionConfig, Intrinsics,
                              decode_depth)
from .safety_state import LatchConfig, Reason, SafetyLatch

# BEST_EFFORT subscriptions are compatible with both reliable and best-effort
# publishers (realsense2_camera's QoS is configurable).
_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

_DETECTION_PARAMS = ('detection_distance', 'min_valid_depth', 'camera_height',
                     'camera_pitch', 'floor_margin', 'floor_height_margin',
                     'corridor_half_width',
                     'roi_top_fraction', 'roi_left_fraction', 'roi_right_fraction')
_LATCH_PARAMS = ('obstacle_pixel_threshold', 'release_pixel_threshold',
                 'release_clear_frames', 'min_hold_time', 'min_usable_fraction',
                 'stale_timeout')
_TF_CHECK_TOLERANCE_M = 0.02


class RealsenseObstacleDetector(Node):

    def __init__(self):
        super().__init__('realsense_obstacle_detector')

        # ── Parameters (names kept from the previous node where they existed) ─
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '')   # '' → derive from depth topic
        self.declare_parameter('depth_scale', 0.001)      # 16UC1 mm → m
        self.declare_parameter('publish_rate', 20.0)      # Hz heartbeat to safety gate
        self.declare_parameter('floor_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_link')

        defaults = DetectionConfig()
        for name in _DETECTION_PARAMS:
            self.declare_parameter(name, float(getattr(defaults, name)))
        latch_defaults = LatchConfig()
        for name in _LATCH_PARAMS:
            value = getattr(latch_defaults, name)
            self.declare_parameter(name, value)

        self._det_cfg = self._build_detection_config({})
        self._latch_cfg = self._build_latch_config({})
        self._depth_scale = float(self.get_parameter('depth_scale').value)
        if not self._depth_scale > 0.0:
            raise ValueError('depth_scale must be > 0')
        rate = float(self.get_parameter('publish_rate').value)
        if not 1.0 <= rate <= 100.0:
            raise ValueError('publish_rate must be in [1, 100] Hz')

        self._depth_topic = str(self.get_parameter('depth_image_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        if not info_topic:
            if not self._depth_topic.endswith('/image_rect_raw'):
                raise ValueError(
                    'camera_info_topic must be set explicitly when depth_image_topic '
                    f'does not end in /image_rect_raw ({self._depth_topic})')
            info_topic = self._depth_topic[:-len('image_rect_raw')] + 'camera_info'
        self._info_topic = info_topic
        self._floor_frame = str(self.get_parameter('floor_frame').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)

        self._intrinsics = None
        self._detector = None
        self._latch = SafetyLatch(self._latch_cfg, time.monotonic())
        self._last_min_dist = math.inf

        # ── Publishers ──────────────────────────────────────────────────────
        self.obstacle_detected_pub = self.create_publisher(
            Bool, '/realsense/obstacle_detected', 10)
        self.min_distance_pub = self.create_publisher(
            Float32, '/realsense/min_obstacle_distance', 10)
        self.alert_pub = self.create_publisher(String, '/realsense/obstacle_alert', 10)

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(CameraInfo, self._info_topic, self._camera_info_cb,
                                 _SENSOR_QOS)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, _SENSOR_QOS)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Heartbeat: re-evaluates staleness and republishes state so the safety
        # gate always holds a fresh value (and can detect this node dying).
        self.create_timer(1.0 / rate, self._heartbeat_cb)

        # One-shot TF cross-check of the configured camera geometry (warning only;
        # the configured, physically measured camera_height is authoritative).
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(3.0, self._tf_cross_check)

        self._log_config()
        self._publish()   # announce BLOCKED (no data yet) immediately

    # ── Configuration ──────────────────────────────────────────────────────

    def _value(self, name, overrides):
        return overrides[name] if name in overrides else self.get_parameter(name).value

    def _build_detection_config(self, overrides) -> DetectionConfig:
        cfg = DetectionConfig(**{n: float(self._value(n, overrides)) for n in _DETECTION_PARAMS})
        cfg.validate()
        return cfg

    def _build_latch_config(self, overrides) -> LatchConfig:
        cfg = LatchConfig(
            obstacle_pixel_threshold=int(self._value('obstacle_pixel_threshold', overrides)),
            release_pixel_threshold=int(self._value('release_pixel_threshold', overrides)),
            release_clear_frames=int(self._value('release_clear_frames', overrides)),
            min_hold_time=float(self._value('min_hold_time', overrides)),
            min_usable_fraction=float(self._value('min_usable_fraction', overrides)),
            stale_timeout=float(self._value('stale_timeout', overrides)),
        )
        cfg.validate()
        return cfg

    def _log_config(self):
        d, la = self._det_cfg, self._latch_cfg
        log = self.get_logger()
        log.info('RealSense blind-spot detector (fail-safe) starting')
        log.info(f'  depth topic       : {self._depth_topic}')
        log.info(f'  camera_info topic : {self._info_topic}')
        log.info(f'  depth band        : {d.min_valid_depth:.2f} .. {d.detection_distance:.2f} m')
        log.info(f'  corridor          : |X| <= {d.corridor_half_width:.2f} m')
        log.info(f'  floor model       : h={d.camera_height:.3f} m '
                 f'pitch={math.degrees(d.camera_pitch):.2f} deg margin={d.floor_margin:.3f} m '
                 f'height_margin={d.floor_height_margin:.3f} m')
        log.info(f'  thresholds        : assert>={la.obstacle_pixel_threshold}px '
                 f'release<={la.release_pixel_threshold}px x{la.release_clear_frames} frames, '
                 f'hold {la.min_hold_time:.2f}s, stale {la.stale_timeout:.2f}s, '
                 f'min usable {la.min_usable_fraction:.2f}')
        log.warn('Forward motion is INHIBITED until valid depth frames are processed')

    def _on_set_parameters(self, params):
        overrides = {p.name: p.value for p in params}
        frozen = {'depth_image_topic', 'camera_info_topic', 'publish_rate', 'depth_scale',
                  'floor_frame', 'camera_frame'} & overrides.keys()
        if frozen:
            return SetParametersResult(
                successful=False, reason=f'{sorted(frozen)} can only be set at start-up')
        try:
            det_cfg = self._build_detection_config(overrides)
            latch_cfg = self._build_latch_config(overrides)
            detector = (DepthObstacleDetector(det_cfg, self._intrinsics)
                        if self._intrinsics is not None else None)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self._det_cfg = det_cfg
        self._detector = detector
        if latch_cfg != self._latch_cfg:
            self._latch_cfg = latch_cfg
            # New thresholds: restart from the safe state.
            self._latch = SafetyLatch(latch_cfg, time.monotonic())
        self.get_logger().info(f'Parameters updated: {sorted(overrides)}')
        return SetParametersResult(successful=True)

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        intr = Intrinsics(width=int(msg.width), height=int(msg.height),
                          fx=float(msg.k[0]), fy=float(msg.k[4]),
                          cx=float(msg.k[2]), cy=float(msg.k[5]))
        if intr == self._intrinsics:
            return
        try:
            detector = DepthObstacleDetector(self._det_cfg, intr)
        except ValueError as exc:
            self.get_logger().error(f'Rejected CameraInfo: {exc}',
                                    throttle_duration_sec=5.0)
            return
        self._intrinsics = intr
        self._detector = detector
        floor = detector.floor_z_per_row
        self.get_logger().info(
            f'Camera intrinsics {intr.width}x{intr.height} fx={intr.fx:.1f} fy={intr.fy:.1f} '
            f'cx={intr.cx:.1f} cy={intr.cy:.1f}; floor Z at bottom row = {floor[-1]:.3f} m')

    def _depth_cb(self, msg: Image):
        now = time.monotonic()
        if self._detector is None:
            self._set_failure(Reason.NO_CAMERA_INFO, now,
                              f'No valid CameraInfo on {self._info_topic} yet')
            return
        try:
            depth_m = decode_depth(msg.encoding, msg.height, msg.width, msg.step,
                                   bool(msg.is_bigendian), msg.data, self._depth_scale)
            result = self._detector.process(depth_m)
        except Exception as exc:  # noqa: BLE001 - any failure must fail safe
            self._set_failure(Reason.ERROR, now, f'Depth frame rejected: {exc}')
            return
        self._last_min_dist = result.min_distance
        was_blocked = self._latch.blocked
        if self._latch.on_frame(result.obstacle_pixels, result.usable_fraction, now):
            self._log_transition(was_blocked, result)
        self._publish()

    def _heartbeat_cb(self):
        was_blocked = self._latch.blocked
        if self._latch.tick(time.monotonic()):
            self._log_transition(was_blocked, None)
        self._publish()

    def _set_failure(self, reason, now, detail):
        was_blocked = self._latch.blocked
        if self._latch.on_failure(reason, now):
            self._log_transition(was_blocked, None)
        self.get_logger().error(detail, throttle_duration_sec=5.0)
        self._publish()

    def _log_transition(self, was_blocked, result):
        latch = self._latch
        detail = ''
        if result is not None:
            dist = ('-' if math.isinf(result.min_distance)
                    else f'{result.min_distance:.2f} m')
            detail = (f' [{result.obstacle_pixels} px, closest {dist}, '
                      f'usable {result.usable_fraction:.0%}]')
        if latch.blocked and not was_blocked:
            self.get_logger().warn(f'BLOCK forward motion: {latch.reason.value}{detail}')
        elif not latch.blocked and was_blocked:
            self.get_logger().info(f'Blind spot CLEAR, forward motion allowed{detail}')
        elif latch.blocked:
            self.get_logger().info(f'Still blocked: {latch.reason.value}{detail}')

    def _publish(self):
        blocked = self._latch.blocked
        self.obstacle_detected_pub.publish(Bool(data=blocked))
        dist = self._last_min_dist
        if self._latch.reason in (Reason.STALE, Reason.NO_DATA, Reason.NO_CAMERA_INFO,
                                  Reason.ERROR):
            dist = math.inf   # distance is unknown, not "none"
        self.min_distance_pub.publish(Float32(data=-1.0 if math.isinf(dist) else float(dist)))
        if blocked:
            self.alert_pub.publish(String(data=f'BLOCKED: {self._latch.reason.value}'))

    def _tf_cross_check(self):
        self._tf_timer.cancel()
        try:
            t = self._tf_buffer.lookup_transform(self._floor_frame, self._camera_frame,
                                                 rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - diagnostic only
            self.get_logger().info(f'TF cross-check skipped ({exc})')
            return
        z = t.transform.translation.z
        if abs(z - self._det_cfg.camera_height) > _TF_CHECK_TOLERANCE_M:
            self.get_logger().warn(
                f'URDF/TF places {self._camera_frame} at z={z:.3f} m above '
                f'{self._floor_frame} but camera_height={self._det_cfg.camera_height:.3f} m. '
                'The floor model uses camera_height (physically measured lens height). '
                'Re-measure the depth-imager height and update the detector configuration if needed.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealsenseObstacleDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
