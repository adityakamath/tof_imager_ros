"""Unit tests for ToFImagerPublisher against a fake sensor, so they run without I2C hardware."""

import numpy as np
import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import PointCloud2
from vl53l5cx.vl53l5cx import VL53L5CXResultsData

from tof_imager_ros import tof_imager_publisher as module


class FakeSensor:
    """Records driver calls and returns canned frames."""

    last = None

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.calls = []
        self.ready = True
        self.ready_error = None
        self.distances = [0] * 64
        self.ranging_data_error = None
        self._i2c_bus = type('Bus', (), {'closed': False, 'close': self._close})()
        FakeSensor.last = self

    def _close(self):
        self._i2c_bus.closed = True

    def init(self):
        self.calls.append('init')

    def set_resolution(self, resolution):
        self.calls.append(('set_resolution', resolution))

    def set_ranging_frequency_hz(self, hz):
        self.calls.append(('set_ranging_frequency_hz', hz))

    def set_ranging_mode(self, mode):
        self.calls.append(('set_ranging_mode', mode))

    def is_alive(self):
        return True

    def start_ranging(self):
        self.calls.append('start_ranging')

    def stop_ranging(self):
        self.calls.append('stop_ranging')

    def check_data_ready(self):
        if self.ready_error:
            raise self.ready_error
        return self.ready

    def get_ranging_data(self):
        if self.ranging_data_error:
            raise self.ranging_data_error
        data = VL53L5CXResultsData(nb_target_per_zone=1)
        data.distance_mm[:len(self.distances)] = self.distances
        return data


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _node(monkeypatch, **params):
    """Create the node with parameter overrides applied before __init__ reads them."""
    monkeypatch.setattr(module, 'ToFImager', FakeSensor)
    original = rclpy.lifecycle.Node.__init__

    def init_with_overrides(self, name, **kwargs):
        original(self, name, parameter_overrides=[
            rclpy.parameter.Parameter(k, value=v) for k, v in params.items()], **kwargs)

    monkeypatch.setattr(rclpy.lifecycle.Node, '__init__', init_with_overrides)
    node = module.ToFImagerPublisher()
    monkeypatch.setattr(rclpy.lifecycle.Node, '__init__', original)
    return node


@pytest.fixture
def node(monkeypatch):
    n = _node(monkeypatch)
    yield n
    n.destroy_node()


def _reference_points(distance_mm, res):
    """The per-pixel loop the vectorized code replaced: x from the row index, y from the column index."""
    out = np.empty((res, res, 3), dtype=np.float32)
    per_px = np.deg2rad(45) / res
    for w in range(res):
        for h in range(res):
            e = max(distance_mm[w][h], 0)
            out[w][h] = [
                -e * np.sin((w + 0.5) * per_px - np.deg2rad(45) / 2) / 1000,
                e * np.sin((h + 0.5) * per_px - np.deg2rad(45) / 2) / 1000,
                e / 1000,
            ]
    return out


class TestConfigure:

    def test_enables_only_the_distance_output(self, node):
        assert node.on_configure(None) == TransitionCallbackReturn.SUCCESS
        kwargs = FakeSensor.last.kwargs
        assert not any(v is False for v in kwargs.values())
        assert set(kwargs) == {
            'disable_ambient_per_spad', 'disable_nb_spads_enabled', 'disable_nb_target_detected',
            'disable_signal_per_spad', 'disable_range_sigma_mm', 'disable_reflectance_percent',
            'disable_target_status', 'disable_motion_indicator'}

    def test_sets_resolution_before_frequency_and_does_not_start_ranging(self, node):
        node.on_configure(None)
        calls = FakeSensor.last.calls
        assert calls.index(('set_resolution', 64)) < calls.index(('set_ranging_frequency_hz', 15))
        assert 'start_ranging' not in calls

    @pytest.mark.parametrize('res, requested, expected', [(8, 30, 15), (8, 10, 10), (4, 100, 60), (4, 30, 30)])
    def test_ranging_frequency_is_capped_per_resolution(self, monkeypatch, res, requested, expected):
        n = _node(monkeypatch, resolution=res, ranging_freq=requested)
        n.on_configure(None)
        assert ('set_ranging_frequency_hz', expected) in FakeSensor.last.calls
        n.destroy_node()

    @pytest.mark.parametrize('params', [{'mode': 2}, {'resolution': 5}])
    def test_invalid_mode_or_resolution_fails(self, monkeypatch, params):
        n = _node(monkeypatch, **params)
        assert n.on_configure(None) == TransitionCallbackReturn.FAILURE
        n.destroy_node()

    def test_sensor_not_alive_fails(self, node, monkeypatch):
        monkeypatch.setattr(FakeSensor, 'is_alive', lambda self: False)
        assert node.on_configure(None) == TransitionCallbackReturn.FAILURE


class TestPointCloud:

    def _configured(self, monkeypatch, **params):
        n = _node(monkeypatch, **params)
        n.on_configure(None)
        return n

    def test_matches_the_per_pixel_reference_at_8x8(self, monkeypatch):
        n = self._configured(monkeypatch)
        distances = np.random.default_rng(0).integers(0, 4000, 64)
        FakeSensor.last.distances = list(distances)
        buf = n.read_sensor()
        np.testing.assert_allclose(buf, _reference_points(distances.reshape(8, 8), 8), atol=1e-5)
        n.destroy_node()

    def test_matches_the_per_pixel_reference_at_4x4(self, monkeypatch):
        n = self._configured(monkeypatch, resolution=4)
        distances = np.random.default_rng(1).integers(0, 4000, 16)
        FakeSensor.last.distances = list(distances) + [0] * 48
        buf = n.read_sensor()
        assert buf.shape == (4, 4, 3)
        np.testing.assert_allclose(buf, _reference_points(distances.reshape(4, 4), 4), atol=1e-5)
        n.destroy_node()

    def test_negative_distances_are_clamped_to_zero(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.distances = [-5] * 64
        np.testing.assert_array_equal(n.read_sensor(), np.zeros((8, 8, 3), dtype=np.float32))
        n.destroy_node()

    def test_x_is_mirrored_and_z_follows_distance(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.distances = [1000] * 64
        buf = n.read_sensor()
        np.testing.assert_allclose(buf[..., 2], 1.0)
        np.testing.assert_allclose(buf[0, :, 0], -buf[7, :, 0], atol=1e-6)  # symmetric across rows, negated
        assert buf[0, 0, 0] > 0 > buf[7, 0, 0]
        n.destroy_node()

    def test_no_frame_ready_returns_none(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.ready = False
        assert n.read_sensor() is None
        n.destroy_node()

    def test_sensor_error_restarts_ranging(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.ready_error = RuntimeError('bus error')
        assert n.read_sensor() is None
        assert FakeSensor.last.calls[-2:] == ['stop_ranging', 'start_ranging']
        n.destroy_node()

    def test_read_error_returns_none(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.ranging_data_error = RuntimeError('read failed')
        assert n.read_sensor() is None
        n.destroy_node()

    def test_short_frame_falls_back_to_zeros(self, monkeypatch):
        n = self._configured(monkeypatch)
        FakeSensor.last.ranging_data_error = IndexError()
        np.testing.assert_array_equal(n.read_sensor(), np.zeros((8, 8, 3), dtype=np.float32))
        n.destroy_node()


class TestLifecycle:

    def _active(self, monkeypatch, **params):
        n = _node(monkeypatch, **params)
        n.on_configure(None)
        assert n.on_activate(None) == TransitionCallbackReturn.SUCCESS
        return n

    def test_activate_starts_ranging(self, monkeypatch):
        n = self._active(monkeypatch)
        assert FakeSensor.last.calls.count('start_ranging') == 1
        n.destroy_node()

    @pytest.mark.parametrize('freq, timer_period, expected', [
        (10, 0.1, 0.05),    # polls twice per ranging period
        (15, 0.1, 0.1 / 3),
        (1, 0.1, 0.1),      # the configured period is the upper bound
    ])
    def test_poll_period_is_at_most_half_the_ranging_period(self, monkeypatch, freq, timer_period, expected):
        n = self._active(monkeypatch, ranging_freq=freq, timer_period=timer_period)
        assert n.timer.timer_period_ns == pytest.approx(expected * 1e9, rel=1e-3)
        n.destroy_node()

    def test_deactivate_stops_ranging_and_clears_entities(self, monkeypatch):
        n = self._active(monkeypatch)
        n.on_deactivate(None)
        assert FakeSensor.last.calls[-1] == 'stop_ranging'
        assert n.timer is None and n.pcl_pub is None
        n.destroy_node()

    def test_reactivation_starts_ranging_again(self, monkeypatch):
        n = self._active(monkeypatch)
        n.on_deactivate(None)
        n.on_activate(None)
        assert FakeSensor.last.calls.count('start_ranging') == 2
        n.destroy_node()

    def test_cleanup_after_deactivate_does_not_stop_twice(self, monkeypatch):
        n = self._active(monkeypatch)
        n.on_deactivate(None)
        sensor = FakeSensor.last
        n.on_cleanup(None)
        assert sensor.calls.count('stop_ranging') == 1
        assert sensor._i2c_bus.closed
        assert n.sensor is None

    def test_shutdown_while_active_stops_ranging_and_closes_bus(self, monkeypatch):
        n = self._active(monkeypatch)
        sensor = FakeSensor.last
        n.on_shutdown(None)
        assert sensor.calls[-1] == 'stop_ranging'
        assert sensor._i2c_bus.closed

    def test_activation_failure_is_reported(self, monkeypatch):
        n = _node(monkeypatch)
        n.on_configure(None)

        def boom():
            raise RuntimeError('no sensor')

        monkeypatch.setattr(FakeSensor.last, 'start_ranging', boom)
        assert n.on_activate(None) == TransitionCallbackReturn.FAILURE
        n.destroy_node()


class TestPublishing:

    def _active(self, monkeypatch):
        n = _node(monkeypatch)
        n.on_configure(None)
        n.on_activate(None)
        FakeSensor.last.distances = [500] * 64
        return n

    def test_does_not_read_the_sensor_without_subscribers(self, monkeypatch):
        n = self._active(monkeypatch)
        reads = []
        monkeypatch.setattr(n, 'read_sensor', lambda: reads.append(1))
        monkeypatch.setattr(n.pcl_pub, 'get_subscription_count', lambda: 0)
        n.publish_pcl()
        assert reads == []
        n.destroy_node()

    def test_publishes_a_point_cloud_when_subscribed(self, monkeypatch):
        n = self._active(monkeypatch)
        published = []
        monkeypatch.setattr(n.pcl_pub, 'get_subscription_count', lambda: 1)
        monkeypatch.setattr(n.pcl_pub, 'publish', published.append)
        n.publish_pcl()
        assert len(published) == 1
        msg = published[0]
        assert isinstance(msg, PointCloud2)
        assert (msg.height, msg.width, msg.point_step, msg.row_step) == (8, 8, 12, 96)
        assert [f.name for f in msg.fields] == ['x', 'y', 'z']
        assert msg.header.frame_id == 'tof_frame'
        assert len(msg.data) == 8 * 8 * 12
        n.destroy_node()

    def test_publishes_nothing_when_no_frame_is_ready(self, monkeypatch):
        n = self._active(monkeypatch)
        FakeSensor.last.ready = False
        published = []
        monkeypatch.setattr(n.pcl_pub, 'get_subscription_count', lambda: 1)
        monkeypatch.setattr(n.pcl_pub, 'publish', published.append)
        n.publish_pcl()
        assert published == []
        n.destroy_node()
