import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from robot_pickup.models import BoundingBox
from robot_pickup.perception.camera import CameraSettings, CameraSource, CameraVision
from robot_pickup.perception.detector import HSVDetector
from robot_pickup.perception.tracking import CentroidTracker, VisionPipeline


def synthetic_frame(offset: int = 0):
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    frame[20:40, 20 + offset:60 + offset] = (0, 0, 255)
    high_red = cv2.cvtColor(np.array([[[176, 240, 220]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
    frame[55:75, 132 + offset:155 + offset] = high_red
    frame[50:80, 84:114] = (0, 255, 0)
    frame[1:4, 1:4] = (0, 0, 255)
    return frame


class PerceptionTests(unittest.TestCase):
    def test_multiple_colors_geometry_and_hue_wraparound(self):
        objects = HSVDetector().detect(synthetic_frame())
        self.assertEqual([obj.label for obj in objects], ["green", "red", "red"])
        left = objects[1]
        self.assertEqual(left.bbox, BoundingBox(20, 20, 60, 40))
        self.assertAlmostEqual(left.cx, -0.6)
        self.assertAlmostEqual(left.cy, -0.4)
        self.assertAlmostEqual(left.area_fraction, 0.04)
        self.assertIsNone(left.confidence)
        self.assertEqual(objects[2].bbox, BoundingBox(132, 55, 155, 75))

    def test_area_filters_and_optional_morphology(self):
        self.assertEqual(HSVDetector(max_area_fraction=0.021).detect(synthetic_frame()), [])
        self.assertEqual(len(HSVDetector(min_area_px=1, morphology_size=0).detect(synthetic_frame())), 4)

    def test_tracking_follows_same_label_objects_through_motion_and_short_gaps(self):
        pipeline = VisionPipeline(HSVDetector(), CentroidTracker(max_gap_s=0.5))
        first = pipeline.process(synthetic_frame(), 1.0).objects
        moved = pipeline.process(synthetic_frame(4), 1.1).objects
        self.assertEqual([obj.object_id for obj in first], [obj.object_id for obj in moved])
        self.assertEqual(len({obj.object_id for obj in moved}), 3)
        self.assertEqual(pipeline.process(np.zeros_like(synthetic_frame()), 1.2).objects, ())
        reappeared = pipeline.process(synthetic_frame(6), 1.3).objects
        self.assertEqual([obj.object_id for obj in moved], [obj.object_id for obj in reappeared])
        expired = pipeline.process(synthetic_frame(7), 1.9).objects
        self.assertTrue({obj.object_id for obj in first}.isdisjoint(obj.object_id for obj in expired))

    def test_recorded_source_delivers_processed_frames_and_ends(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as folder:
            path = Path(folder) / "generated.avi"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (200, 100))
            self.assertTrue(writer.isOpened())
            try:
                for offset in range(4):
                    writer.write(synthetic_frame(offset))
            finally:
                writer.release()
            source = CameraSource.parse(str(path))
            self.assertFalse(source.is_live)
            worker = CameraVision(source, CameraSettings(), VisionPipeline(HSVDetector(), CentroidTracker()))
            worker.start()
            try:
                self.assertTrue(worker.finished.wait(4))
                self.assertIsNone(worker.error)
                self.assertIsNotNone(worker.latest)
                self.assertEqual(len(worker.latest.scene.objects), 3)
            finally:
                worker.close()

    def test_live_source_parsing(self):
        self.assertEqual(CameraSource.parse("0").address, 0)
        self.assertTrue(CameraSource.parse("0").is_live)
        self.assertTrue(CameraSource.parse("http://192.168.1.50:81/stream").is_stream)
        with self.assertRaises(ValueError):
            CameraSource.parse("http://")
