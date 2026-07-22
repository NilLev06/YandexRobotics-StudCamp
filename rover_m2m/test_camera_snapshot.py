import sys
import types
import unittest
from unittest.mock import patch

import camera_snapshot as camera


class Encoded:
    def tobytes(self):
        return b"jpeg-data"


class FakeCapture:
    def __init__(self, opens=True, reads=True):
        self.opens = opens
        self.reads = reads
        self.released = False

    def set(self, _property, _value):
        return True

    def open(self, _url, _backend):
        return self.opens

    def read(self):
        return self.reads, object() if self.reads else None

    def release(self):
        self.released = True


def fake_cv2(capture):
    return types.SimpleNamespace(
        CAP_FFMPEG=1900,
        CAP_PROP_OPEN_TIMEOUT_MSEC=53,
        CAP_PROP_READ_TIMEOUT_MSEC=54,
        IMWRITE_JPEG_QUALITY=1,
        VideoCapture=lambda: capture,
        imencode=lambda *_args: (True, Encoded()),
    )


class CameraSnapshotTests(unittest.TestCase):
    def test_captures_and_releases_jpeg(self):
        capture = FakeCapture()
        with patch.dict(sys.modules, {"cv2": fake_cv2(capture)}):
            self.assertEqual(camera.capture_jpeg("rtsp://camera", 80, 1000), b"jpeg-data")
        self.assertTrue(capture.released)

    def test_open_failure_is_reported_and_released(self):
        capture = FakeCapture(opens=False)
        with patch.dict(sys.modules, {"cv2": fake_cv2(capture)}):
            with self.assertRaisesRegex(camera.CameraSnapshotError, "cannot open"):
                camera.capture_jpeg("rtsp://camera", 80, 1000)
        self.assertTrue(capture.released)

    def test_invalid_quality_from_environment_is_rejected(self):
        with patch.dict(camera.os.environ, {"CAMERA_SNAPSHOT_JPEG_QUALITY": "101"}):
            with self.assertRaisesRegex(camera.CameraSnapshotError, "between 1 and 100"):
                camera.capture_jpeg(timeout_ms=1000)


if __name__ == "__main__":
    unittest.main()
