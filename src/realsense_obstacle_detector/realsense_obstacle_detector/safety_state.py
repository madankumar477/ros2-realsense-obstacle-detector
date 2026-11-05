"""
Fail-safe obstacle latch for the RealSense blind-spot detector.

Pure Python (no rclpy) so it can be tested offline with an injected clock.

Policy
------
* The latch starts BLOCKED: no camera data has been seen yet, so the blind
  spot is unknown, not clear.
* Any unsafe condition blocks immediately (no debounce on the safe side):
    OBSTACLE      >= obstacle_pixel_threshold pixels of obstacle evidence
    BLIND         usable-pixel fraction below min_usable_fraction (lens
                  covered, object closer than the sensor's minimum range,
                  sensor fault) -- zero/invalid depth is never "clear"
    STALE         no depth frame for longer than stale_timeout
    NO_DATA       no depth frame received since start-up
    NO_CAMERA_INFO / ERROR   intrinsics missing or frame processing failed
* Release requires release_clear_frames *consecutive* clean frames
  (obstacle_pixels <= release_pixel_threshold) AND at least min_hold_time
  since the last unsafe observation. Frames between the release and assert
  thresholds hold the current state (hysteresis), preventing stop/go chatter.
"""

from dataclasses import dataclass
from enum import Enum
import math


class Reason(str, Enum):
    CLEAR = 'clear'
    OBSTACLE = 'obstacle in blind spot'
    BLIND = 'insufficient valid depth (camera occluded / object inside sensor minimum range)'
    STALE = 'depth stream stale'
    NO_DATA = 'no depth frame received yet'
    NO_CAMERA_INFO = 'no valid CameraInfo yet'
    ERROR = 'depth frame processing error'
    RELEASING = 'obstacle gone, waiting for release confirmation'


@dataclass(frozen=True)
class LatchConfig:
    obstacle_pixel_threshold: int = 20
    release_pixel_threshold: int = 10
    release_clear_frames: int = 5
    min_hold_time: float = 0.5
    min_usable_fraction: float = 0.20
    stale_timeout: float = 0.5

    def validate(self) -> None:
        if self.obstacle_pixel_threshold < 1:
            raise ValueError('obstacle_pixel_threshold must be >= 1')
        if not 0 <= self.release_pixel_threshold < self.obstacle_pixel_threshold:
            raise ValueError(
                'require 0 <= release_pixel_threshold < obstacle_pixel_threshold')
        if self.release_clear_frames < 1:
            raise ValueError('release_clear_frames must be >= 1')
        if self.min_hold_time < 0.0 or not math.isfinite(self.min_hold_time):
            raise ValueError('min_hold_time must be a finite value >= 0')
        if not 0.0 <= self.min_usable_fraction <= 1.0:
            raise ValueError('min_usable_fraction must be in [0, 1]')
        if not 0.0 < self.stale_timeout < 10.0:
            raise ValueError('stale_timeout must be in (0, 10) s')


class SafetyLatch:
    """Tracks whether forward motion must be inhibited. Times are seconds
    from a monotonic clock supplied by the caller."""

    def __init__(self, config: LatchConfig, now: float):
        config.validate()
        self.config = config
        self.blocked = True
        self.reason = Reason.NO_DATA
        self._last_frame_time = None
        self._last_unsafe_time = now
        self._clear_streak = 0

    def _unsafe(self, reason: Reason, now: float) -> bool:
        changed = not self.blocked or self.reason != reason
        self.blocked = True
        self.reason = reason
        self._last_unsafe_time = now
        self._clear_streak = 0
        return changed

    def on_frame(self, obstacle_pixels: int, usable_fraction: float, now: float) -> bool:
        """Feed one processed frame. Returns True if the state changed."""
        self._last_frame_time = now
        cfg = self.config
        if not usable_fraction >= cfg.min_usable_fraction:   # also catches NaN
            return self._unsafe(Reason.BLIND, now)
        if obstacle_pixels >= cfg.obstacle_pixel_threshold:
            return self._unsafe(Reason.OBSTACLE, now)
        if not self.blocked:
            return False
        if obstacle_pixels > cfg.release_pixel_threshold:
            # Hysteresis band: keep blocking, restart the clean-frame count.
            self._clear_streak = 0
            return False
        self._clear_streak += 1
        if (self._clear_streak >= cfg.release_clear_frames
                and now - self._last_unsafe_time >= cfg.min_hold_time):
            self.blocked = False
            self.reason = Reason.CLEAR
            self._clear_streak = 0
            return True
        changed = self.reason != Reason.RELEASING
        self.reason = Reason.RELEASING
        return changed

    def on_failure(self, reason: Reason, now: float) -> bool:
        """A frame (or its prerequisites) could not be evaluated."""
        return self._unsafe(reason, now)

    def tick(self, now: float) -> bool:
        """Periodic staleness check. Returns True if the state changed."""
        if self._last_frame_time is None:
            if self.reason in (Reason.NO_CAMERA_INFO, Reason.ERROR):
                return self._unsafe(self.reason, now)
            return self._unsafe(Reason.NO_DATA, now)
        if now - self._last_frame_time > self.config.stale_timeout:
            return self._unsafe(Reason.STALE, now)
        return False
