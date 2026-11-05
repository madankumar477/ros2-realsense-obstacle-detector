import pytest

from realsense_obstacle_detector.safety_state import LatchConfig, Reason, SafetyLatch

CFG = LatchConfig()   # 20/10 px, 5 frames, 0.5 s hold, 0.2 usable, 0.5 s stale
DT = 1.0 / 30.0       # D435i depth at 30 fps


def feed(latch, t, n, px=0, usable=0.8):
    for _ in range(n):
        t += DT
        latch.on_frame(px, usable, t)
    return t


def test_starts_blocked_until_frames_arrive():
    latch = SafetyLatch(CFG, 0.0)
    assert latch.blocked and latch.reason == Reason.NO_DATA
    latch.tick(0.2)
    assert latch.blocked


def test_releases_after_clean_frames_and_hold():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 5)               # 5 clean frames but only 0.17 s
    assert latch.blocked and latch.reason == Reason.RELEASING
    t = feed(latch, t, 12)                # past 0.5 s hold
    assert not latch.blocked and latch.reason == Reason.CLEAR


def test_obstacle_blocks_immediately():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    assert not latch.blocked
    latch.on_frame(25, 0.8, t + DT)
    assert latch.blocked and latch.reason == Reason.OBSTACLE


def test_hysteresis_band_holds_block_and_does_not_assert_when_clear():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    t = feed(latch, t, 10, px=15)         # between 10 and 20: stays clear
    assert not latch.blocked
    t = feed(latch, t, 1, px=20)
    assert latch.blocked
    t = feed(latch, t, 60, px=15)         # in the band: never releases
    assert latch.blocked
    t = feed(latch, t, 4, px=10)
    assert latch.blocked                  # streak 4 < 5
    feed(latch, t, 1, px=10)
    assert not latch.blocked


def test_flicker_does_not_release():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    t = feed(latch, t, 1, px=50)
    for _ in range(20):                   # alternate clean / obstacle
        t = feed(latch, t, 3, px=0)
        t = feed(latch, t, 1, px=50)
        assert latch.blocked


def test_blind_camera_blocks():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    latch.on_frame(0, 0.05, t + DT)
    assert latch.blocked and latch.reason == Reason.BLIND
    latch.on_frame(0, float('nan'), t + 2 * DT)
    assert latch.blocked


def test_stale_stream_blocks_and_needs_fresh_streak():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    assert not latch.blocked
    latch.tick(t + 0.4)
    assert not latch.blocked
    latch.tick(t + 0.6)
    assert latch.blocked and latch.reason == Reason.STALE
    t = feed(latch, t + 0.6, 2)
    assert latch.blocked                  # needs 5 frames and 0.5 s again
    feed(latch, t, 20)
    assert not latch.blocked


def test_failures_block():
    latch = SafetyLatch(CFG, 0.0)
    t = feed(latch, 0.0, 30)
    latch.on_failure(Reason.ERROR, t)
    assert latch.blocked and latch.reason == Reason.ERROR


def test_no_camera_info_keeps_blocking_reason():
    latch = SafetyLatch(CFG, 0.0)
    latch.on_failure(Reason.NO_CAMERA_INFO, 0.1)
    latch.tick(0.2)
    assert latch.blocked and latch.reason == Reason.NO_CAMERA_INFO


@pytest.mark.parametrize('kwargs', [
    dict(release_pixel_threshold=20),
    dict(obstacle_pixel_threshold=0),
    dict(release_clear_frames=0),
    dict(min_usable_fraction=1.5),
    dict(stale_timeout=0.0),
])
def test_invalid_latch_config(kwargs):
    with pytest.raises(ValueError):
        SafetyLatch(LatchConfig(**kwargs), 0.0)
