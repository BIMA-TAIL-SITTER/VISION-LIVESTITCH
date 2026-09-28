import io
import json
import os
import sys
import unittest
import piexif

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.flight_metadata import (
    extract_flight_metadata, build_data_matrix, GPSDistanceFilter,
    AttitudeThresholdFilter,
)

def _make_test_jpeg_with_exif(path, lat, lon, alt_m, roll_deg, pitch_deg, yaw_deg):
    """Mirrors PROGRAM_JUANG/PROGRAM-SENDER1/sender.py's embed_flight_metadata_exif()."""
    import numpy as np
    import cv2

    blank = (np.zeros((10, 10, 3)) + 128).astype(np.uint8)
    ok, buf = cv2.imencode('.jpg', blank)
    jpeg_bytes = buf.tobytes()

    def _deg_to_dms_rational(deg_float):
        """Convert decimal degrees to degrees, minutes, seconds in rational format."""
        deg_abs = abs(deg_float)
        d = int(deg_abs)
        m_float = (deg_abs - d) * 60
        m = int(m_float)
        m = min(m, 59)  # Ensure minutes are less than 60
        s_float = (m_float - m) * 60
        s = int(s_float * 10000)  # Convert to rational by multiplying by 10000
        return ((d, 1), (m, 1), (s, 10000))
    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSLatitudeRef: 'S' if lat < 0 else 'N',
        piexif.GPSIFD.GPSLatitude: _deg_to_dms_rational(lat),
        piexif.GPSIFD.GPSLongitudeRef: 'W' if lon < 0 else 'E',
        piexif.GPSIFD.GPSLongitude: _deg_to_dms_rational(lon),
        piexif.GPSIFD.GPSAltitudeRef: 0 if alt_m >= 0 else 1,
        piexif.GPSIFD.GPSAltitude: (int(round(abs(alt_m) * 100)), 100),
        piexif.GPSIFD.GPSImgDirectionRef: 'T',
        piexif.GPSIFD.GPSImgDirection: (int(round((yaw_deg % 360) * 100)), 100),
    }
    attitude_json = json.dumps({
        "roll": round(roll_deg, 2),
        "pitch": round(pitch_deg, 2),
        "yaw": round(yaw_deg, 2)
    })
    zeroth_ifd = {piexif.ImageIFD.ImageDescription: attitude_json.encode('utf-8')}
    exif_bytes = piexif.dump({ "GPS": gps_ifd, "0th": zeroth_ifd})

    out = io.BytesIO()
    piexif.insert(exif_bytes, jpeg_bytes, out)
    with open(path, 'wb') as f:
        f.write(out.getvalue())

class TestExtractFlightMetadata(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = os.path.join(os.path.dirname(__file__), 'tmp_flight_metadata')
        os.makedirs(self.tmp_dir, exist_ok=True)

    def test_extracts_known_gps_and_attitude_in_degrees(self):
        path = os.path.join(self.tmp_dir, 'test_image.jpg')
        _make_test_jpeg_with_exif(
            path, lat=-7.250445, lon=112.768845, alt_m=80.5,
            roll_deg=12.3, pitch_deg=-4.5, yaw_deg=271.8,
        )

        result = extract_flight_metadata(path)

        self.assertTrue(result["has_telemetry"])
        self.assertAlmostEqual(result["latitude"], -7.250445, places=4)
        self.assertAlmostEqual(result["longitude"], 112.768845, places=4)
        self.assertAlmostEqual(result["altitude"], 80.5, places=1)
        self.assertAlmostEqual(result["roll"], 12.3, places=1)
        self.assertAlmostEqual(result["pitch"], -4.5, places=1)
        self.assertAlmostEqual(result["yaw"], 271.8, places=1)

    def test_missing_exif_returns_pose_not_fabricated_gps(self):
        import numpy as np
        import cv2
        path = os.path.join(self.tmp_dir, 'test_image_no_exif.jpg')
        blank = (np.zeros((10, 10, 3)) + 128).astype(np.uint8)
        cv2.imwrite(path, blank)

        result = extract_flight_metadata(path)

        self.assertFalse(result["has_telemetry"])
        self.assertEqual(result["latitude"], 0.0)
        self.assertEqual(result["longitude"], 0.0)
        self.assertEqual(result["roll"], 0.0)
        self.assertEqual(result["pitch"], 0.0)
        self.assertEqual(result["yaw"], 0.0)

class TestBuildDataMatrix(unittest.TestCase):
    def test_first_entry_is_origin_with_correct_attitude_columns(self):
        metadata = [
            {"latitude": -7.250445, "longitude": 112.768845, "altitude": 80.0,
             "yaw": 90.0, "pitch": 2.0, "roll": -1.0},
            {"latitude": -7.250400, "longitude": 112.768900, "altitude": 80.0,
             "yaw": 90.0, "pitch": 1.5, "roll": 0.5},
        ]

        matrix = build_data_matrix(metadata)

        self.assertEqual(matrix.shape, (2, 6))
        # First entry is the origin -> local X/Y must be 0,0
        self.assertAlmostEqual(matrix[0, 0], 0.0, places=6)
        self.assertAlmostEqual(matrix[0, 1], 0.0, places=6)
        self.assertAlmostEqual(matrix[0, 2], 80.0, places=6)
        # Attitude columns carried through in DEGREES, unmodified
        self.assertAlmostEqual(matrix[0, 3], 90.0, places=6)
        self.assertAlmostEqual(matrix[0, 4], 2.0, places=6)
        self.assertAlmostEqual(matrix[0, 5], -1.0, places=6)
        self.assertAlmostEqual(matrix[1, 3], 90.0, places=6)
        self.assertAlmostEqual(matrix[1, 4], 1.5, places=6)
        self.assertAlmostEqual(matrix[1, 5], 0.5, places=6)
        # Second entry moved north-east -> both local X and Y should be positive
        self.assertGreater(matrix[1, 0], 0.0)
        self.assertGreater(matrix[1, 1], 0.0)

class TestGPSDistanceFilter(unittest.TestCase):
    def test_first_call_always_accepted(self):
        f = GPSDistanceFilter(threshold_m=3.0)
        accepted, dist = f.should_accept(-7.250445, 112.768845)
        self.assertTrue(accepted)
        self.assertEqual(dist, 0.0)

    def test_rejects_when_too_close_to_last_accepted(self):
        f = GPSDistanceFilter(threshold_m=50.0)
        f.should_accept(-7.250445, 112.768845)
        # ~1 meter away -- well under the 50m threshold
        accepted, dist = f.should_accept(-7.250446, 112.768846)
        self.assertFalse(accepted)
        self.assertLess(dist, 50.0)

    def test_accepts_when_far_enough_and_updates_reference(self):
        f = GPSDistanceFilter(threshold_m=3.0)
        f.should_accept(-7.250445, 112.768845)
        # ~100m north -- well over the 3m threshold
        accepted, dist = f.should_accept(-7.249545, 112.768845)
        self.assertTrue(accepted)
        self.assertGreater(dist, 3.0)

    def test_two_instances_do_not_share_state(self):
        f1 = GPSDistanceFilter(threshold_m=1000.0)
        f2 = GPSDistanceFilter(threshold_m=1000.0)
        f1.should_accept(-7.250445, 112.768845)
        # f2 has never seen a position -- must accept unconditionally,
        # regardless of what f1 saw
        accepted, dist = f2.should_accept(10.0, 20.0)
        self.assertTrue(accepted)
        self.assertEqual(dist, 0.0)

class TestAttitudeThresholdFilter(unittest.TestCase):
    def test_accepts_when_under_threshold(self):
        f = AttitudeThresholdFilter(threshold_deg=30.0)
        accepted, worst = f.should_accept(roll_deg=10.0, pitch_deg=-5.0)
        self.assertTrue(accepted)
        self.assertEqual(worst, 10.0)

    def test_rejects_when_roll_exceeds_threshold(self):
        f = AttitudeThresholdFilter(threshold_deg=30.0)
        accepted, worst = f.should_accept(roll_deg=45.0, pitch_deg=0.0)
        self.assertFalse(accepted)
        self.assertEqual(worst, 45.0)

    def test_rejects_when_pitch_exceeds_threshold(self):
        f = AttitudeThresholdFilter(threshold_deg=30.0)
        accepted, worst = f.should_accept(roll_deg=0.0, pitch_deg=-35.0)
        self.assertFalse(accepted)
        self.assertEqual(worst, 35.0)


if __name__ == '__main__':
    unittest.main()
