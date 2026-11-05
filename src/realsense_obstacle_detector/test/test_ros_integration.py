"""
Offline end-to-end test: RealSense detector node → cmd_vel_safety_gate node.

Nothing is ever published to DDS: Node.create_publisher is replaced by a
recorder before any node is built, nodes are never spun, and the test forces
an isolated domain with ROS_LOCALHOST_ONLY=1. Callbacks are invoked directly
with synthetic messages and an injected monotonic clock.
"""

import os
import sys

os.environ['ROS_DOMAIN_ID'] = '199'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import pytest  # noqa: E402

rclpy = pytest.importorskip('rclpy')
gate_mod = pytest.importorskip('cmd_vel_safety_gate.safety_gate_node')

from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

import realsense_obstacle_detector.realsense_obstacle_detector as det_mod  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from synthetic import D435I, add_box, render_floor, to_16uc1_bytes  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class Recorder:
    def __init__(self, topic):
        self.topic = topic
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def last(self):
        return self.msgs[-1]


@pytest.fixture
def env(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(det_mod, 'time', clock)
    monkeypatch.setattr(gate_mod, 'time', clock)
    pubs = {}

    def fake_create_publisher(self, msg_type, topic, qos, **kwargs):
        rec = Recorder(topic)
        pubs[(self.get_name(), topic)] = rec
        return rec

    monkeypatch.setattr(rclpy.node.Node, 'create_publisher', fake_create_publisher)
    rclpy.init()
    nodes = []
    yield clock, pubs, nodes
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


def camera_info():
    m = CameraInfo()
    m.width, m.height = D435I.width, D435I.height
    m.k = [D435I.fx, 0.0, D435I.cx, 0.0, D435I.fy, D435I.cy, 0.0, 0.0, 1.0]
    return m


def depth_msg(depth_m):
    m = Image()
    m.height, m.width = depth_m.shape
    m.encoding = '16UC1'
    data, step = to_16uc1_bytes(depth_m)
    m.step, m.data = step, data
    return m


def fwd(x=0.3, wz=0.2):
    t = Twist()
    t.linear.x, t.angular.z = x, wz
    return t


def make_gate(nodes, timeout):
    """Build the unmodified gate class with realsense_timeout_sec overridden,
    exactly as the launch file would pass it."""
    orig_init = rclpy.node.Node.__init__

    def init_with_override(self, name, **kwargs):
        kwargs.setdefault('parameter_overrides',
                          [Parameter('realsense_timeout_sec', value=timeout)])
        orig_init(self, name, **kwargs)

    rclpy.node.Node.__init__ = init_with_override
    try:
        gate = gate_mod.SafetyGateNode()
    finally:
        rclpy.node.Node.__init__ = orig_init
    nodes.append(gate)
    return gate


def step(clock, det, gate, depth=None, dt=1.0 / 30.0):
    """Advance time, deliver a frame (if any), run timers, relay Bool to gate."""
    clock.t += dt
    if depth is not None:
        det._depth_cb(depth_msg(depth))
    det._heartbeat_cb()
    gate._realsense_cb(det.obstacle_detected_pub.last)
    gate._heartbeat_cb()


def gate_out(gate, cmd):
    gate._cmd_vel_cb(cmd)
    return gate._pub.last


def test_camera_obstacle_inhibits_forward_motion_end_to_end(env):
    clock, pubs, nodes = env
    det = det_mod.RealsenseObstacleDetector()
    nodes.append(det)
    gate = make_gate(nodes, 1.0)

    # Start-up: detector announces BLOCKED, gate watchdog also blocks.
    assert det.obstacle_detected_pub.last.data is True
    out = gate_out(gate, fwd())
    assert out.linear.x == 0.0 and out.angular.z == pytest.approx(0.2)

    # Depth before CameraInfo stays blocked.
    step(clock, det, gate, render_floor())
    assert det.obstacle_detected_pub.last.data is True

    det._camera_info_cb(camera_info())
    floor = render_floor(noise=0.003)
    for _ in range(25):                         # ~0.8 s of clean frames
        step(clock, det, gate, floor)
    assert det.obstacle_detected_pub.last.data is False
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)

    # Obstacle appears 0.45 m ahead → next frame blocks forward motion.
    box = add_box(floor, z=0.45, x_left=-0.15, x_right=0.15, top=0.25)
    step(clock, det, gate, box)
    assert det.obstacle_detected_pub.last.data is True
    assert det.min_distance_pub.last.data == pytest.approx(0.45, abs=0.01)
    out = gate_out(gate, fwd())
    assert out.linear.x == 0.0
    assert out.angular.z == pytest.approx(0.2)      # rotation still allowed
    assert gate_out(gate, fwd(x=-0.1)).linear.x == pytest.approx(-0.1)  # reverse allowed

    # Obstacle leaves: needs 5 clean frames AND 0.5 s.
    for _ in range(5):
        step(clock, det, gate, floor)
    assert det.obstacle_detected_pub.last.data is True
    for _ in range(12):
        step(clock, det, gate, floor)
    assert det.obstacle_detected_pub.last.data is False
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)

    # Camera driver dies: detector heartbeat goes stale → blocked.
    step(clock, det, gate, None, dt=0.6)
    assert det.obstacle_detected_pub.last.data is True
    assert gate_out(gate, fwd()).linear.x == 0.0

    # Malformed frame → blocked.
    for _ in range(25):
        step(clock, det, gate, floor)
    assert det.obstacle_detected_pub.last.data is False
    bad = depth_msg(floor)
    bad.encoding = 'rgb8'
    clock.t += 0.03
    det._depth_cb(bad)
    assert det.obstacle_detected_pub.last.data is True


def test_gate_watchdog_blocks_when_detector_silent(env):
    clock, pubs, nodes = env
    gate = make_gate(nodes, 1.0)
    gate._realsense_cb(Bool(data=False))
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)
    clock.t += 1.2                       # detector crashed: no messages
    gate._heartbeat_cb()
    assert gate_out(gate, fwd()).linear.x == 0.0
    gate._realsense_cb(Bool(data=False))  # detector back
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)


def test_gate_default_behaviour_unchanged_without_watchdog(env):
    clock, pubs, nodes = env
    gate = gate_mod.SafetyGateNode()      # default realsense_timeout_sec = 0.0
    nodes.append(gate)
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)
    clock.t += 100.0
    gate._heartbeat_cb()
    assert gate_out(gate, fwd()).linear.x == pytest.approx(0.3)
    gate._realsense_cb(Bool(data=True))
    assert gate_out(gate, fwd()).linear.x == 0.0
    gate._livox_cb(Bool(data=True))
    out = gate_out(gate, fwd())
    assert out.linear.x == 0.0 and out.angular.z == 0.0


def test_live_parameter_validation(env):
    clock, pubs, nodes = env
    det = det_mod.RealsenseObstacleDetector()
    nodes.append(det)
    det._camera_info_cb(camera_info())
    bad = det.set_parameters([Parameter('min_valid_depth', value=0.9)])
    assert not bad[0].successful
    ok = det.set_parameters([Parameter('floor_margin', value=0.04)])
    assert ok[0].successful and det._det_cfg.floor_margin == 0.04
    frozen = det.set_parameters([Parameter('depth_image_topic', value='/x/image_rect_raw')])
    assert not frozen[0].successful
