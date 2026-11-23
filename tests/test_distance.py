import unittest
from dataclasses import replace

from robot_pickup.models import BoundingBox, Detection, DistanceReading, DistanceSample, Scene
from robot_pickup.sensors.association import BeamAssociation, BeamSettings
from robot_pickup.sensors.distance import DistanceSettings, FilteredDistance, HTTPDistance


class MutableDistance:
    sample: DistanceSample | None = None

    def latest(self, now: float) -> DistanceSample | None:
        return self.sample


class DistanceTests(unittest.TestCase):
    def setUp(self):
        self.provider = MutableDistance()
        self.distance = FilteredDistance(self.provider, DistanceSettings(max_age_s=0.5, median_window=3))

    def test_median_uses_unique_samples_and_preserves_latest_raw_echo(self):
        for timestamp, cm in ((0.0, 30), (0.1, 29), (0.2, 4)):
            self.provider.sample = DistanceSample(cm, timestamp)
            reading = self.distance.latest(timestamp)
        self.assertEqual(reading.sample.cm, 4)
        self.assertEqual(reading.filtered_cm, 29)
        self.assertEqual(self.distance.latest(0.3).filtered_cm, 29)

    def test_invalid_stale_and_future_data_do_not_reuse_previous_median(self):
        self.assertFalse(self.distance.latest(1.0).valid)
        self.provider.sample = DistanceSample(20, 1.0)
        self.assertTrue(self.distance.latest(1.0).valid)
        self.assertFalse(self.distance.latest(1.6).valid)
        self.provider.sample = DistanceSample(None, 1.6, False, "no echo")
        self.assertEqual(self.distance.latest(1.6).reason, "no echo")
        for index, cm in enumerate((1, 500, True, float("nan"), None), start=1):
            now = 1.6 + index / 100
            self.provider.sample = DistanceSample(cm, now)
            self.assertFalse(self.distance.latest(now).valid)
        self.provider.sample = DistanceSample(22, 2.0)
        self.assertFalse(self.distance.latest(1.9).valid)
        self.assertEqual(self.distance.latest(2.0).filtered_cm, 22)

    def test_out_of_order_sample_stays_invalid_when_polled_again(self):
        self.provider.sample = DistanceSample(20, 0.2)
        self.assertTrue(self.distance.latest(0.2).valid)
        self.provider.sample = DistanceSample(30, 0.1)
        for now in (0.3, 0.4):
            self.assertIn("out of order", self.distance.latest(now).reason)
        self.provider.sample = DistanceSample(40, 0.4)
        self.assertEqual(self.distance.latest(0.4).filtered_cm, 40)

    def test_http_timestamp_is_translated_and_a_cached_response_goes_stale(self):
        provider = HTTPDistance("http://127.0.0.1/distance")
        payload = {"distance_cm": 25, "valid": True, "measured_at": 999.75}
        provider._receive(payload, now=10.0, wall_time=1000.0)
        sample = provider.latest(10.0)
        self.assertEqual(sample, DistanceSample(25.0, 9.75))
        provider._receive(payload, now=10.5, wall_time=1000.5)
        self.assertEqual(provider.latest(10.5), sample)
        self.assertFalse(FilteredDistance(provider, DistanceSettings()).latest(10.5).valid)
        with self.assertRaises(ValueError):
            provider._receive({**payload, "measured_at": 1001.0}, now=10.5, wall_time=1000.5)


class BeamTests(unittest.TestCase):
    def test_requires_calibration_a_center_hit_and_a_clear_coherent_beam(self):
        target = replace(Detection.from_box("red", BoundingBox(42, 40, 58, 60), 100, 100), object_id=1)
        blocker = replace(Detection.from_box("green", BoundingBox(55, 45, 67, 53), 100, 100), object_id=2)
        elsewhere = replace(Detection.from_box("green", BoundingBox(75, 45, 87, 53), 100, 100), object_id=3)
        off_axis = replace(Detection.from_box("red", BoundingBox(51, 45, 58, 60), 100, 100), object_id=1)
        scene = Scene(1.0, 100, 100, (target, elsewhere))
        reading = DistanceReading(DistanceSample(21, 1.0), 21, "")
        beam = BeamAssociation(BeamSettings(calibrated=True))
        self.assertEqual(beam.estimate(target, scene, reading), 21)
        self.assertIsNone(BeamAssociation(BeamSettings()).estimate(target, scene, reading))
        self.assertIsNone(beam.estimate(target, replace(scene, objects=(target, blocker)), reading))
        self.assertIsNone(beam.estimate(off_axis, replace(scene, objects=(off_axis,)), reading))
        self.assertIsNone(beam.estimate(target, replace(scene, observed_at=1.25), reading))
        self.assertIsNone(beam.estimate(target, scene, DistanceReading()))
