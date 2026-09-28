# Composable Orchestrator Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `src/flight_metadata.py` — a correct, composable EXIF/GPS/attitude extraction and filtering module — and wire it into `service.py` so the local simulation harness stitches with real degrees-based attitude data, GPS-distance deduplication, and attitude-tilt gating, instead of the current always-zero attitude and zero filtering.

**Architecture:** One new standalone module (`src/flight_metadata.py`) holding two pure functions (`extract_flight_metadata`, `build_data_matrix`) and two small stateful filter classes (`GPSDistanceFilter`, `AttitudeThresholdFilter`), each independently unit-testable with no FastAPI/watchdog dependency. `service.py` then wires one filter pair per `StitchingSession` (not global — sessions must not share filter state) and switches its image-admission and stitch-triggering logic from "any new file counts" to "only images that pass the filters (or explicitly flagged degraded-confidence fallback) get accepted and stitched."

**Tech Stack:** Python 3, `exifread` (already a dependency), `piexif` (new, test-fixture generation only), `numpy`, `pytest`/`unittest` (repo already uses `unittest.TestCase`, see `test/test_extract_gps.py`).

## Global Constraints

- `geometry.computeUnRotMatrix` is NOT modified — it already correctly expects degrees (confirmed against `src/ImageMosaic.py`'s own `# Yaw (degrees)` comments, see `docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md`). `extract_flight_metadata` must never convert to radians — that was the bug.
- `src/Combiner.py` is NOT modified in this plan. GPS-sanity-check / RANSAC-inlier-ratio work is a separate, already-tracked effort (`docs/DRIFT_MISREGISTRATION.md` §9) — must be tested independently so results aren't conflated (explicit user decision, same session).
- `PROGRAM_JUANG/PROGRAM-SENDER1/` is NOT modified. It's the colleague's PoC/R&D fork; the new module is a from-scratch, correct reimplementation informed by reading that code, not a patch to it.
- `receiver_socket.py` and `sender_socket_sim.py` wire protocols are NOT modified.
- No fabricated GPS data anywhere — missing telemetry means `has_telemetry: False` + zero pose, never invented coordinates.
- Filter instances are per-`StitchingSession`, never module-level globals (multi-UAV sessions must not share state).
- Attitude gate is checked before the GPS-distance gate, always (preserves `stitcher.py`'s deliberate ordering — swapping it lets a tilt-rejected frame still mutate the GPS filter's reference position).
- OpenCV-`Stitcher` fallback is explicitly OUT of scope (dropped — breaks GPS-based overlay georeferencing).

---

## Execution Approach

Three options for working through the tasks below. Recorded here so it's easy to switch later without re-deriving them.

1. **Subagent-Driven** (`superpowers:subagent-driven-development`) — a fresh subagent per task, review between tasks, fast iteration. Best when you want tasks done quickly and reviewed in batches.
2. **Inline Execution** (`superpowers:executing-plans`) — executed in the current chat session, batch execution with checkpoints for review. Best when you want to stay close to the work without doing the typing yourself.
3. **Solo manual** (chosen for this round) — user writes the code, debugs, and tests each task themselves, using this plan as the reference spec (exact file paths, exact code, exact test commands, exact expected output for every step). Chosen deliberately as a learning exercise, not because 1/2 are unavailable.

**If you hit your limit partway through** (fatigue, stuck on a task, want a second pair of hands) — just say so, and switch to option 1 or 2 for the remaining tasks. Already-completed tasks don't need to be redone; whichever option picks up next just continues from the first unchecked task below. No need to re-plan or re-explain context — this file is the shared reference regardless of which option executes it.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `src/flight_metadata.py` | Create | `extract_flight_metadata()`, `build_data_matrix()`, `GPSDistanceFilter`, `AttitudeThresholdFilter`, `haversine_m()` |
| `test/test_flight_metadata.py` | Create | Unit tests for all four pieces above, using synthetic EXIF fixtures (no external dataset dependency) |
| `test/test_geometry_unrot.py` | Create | Regression test locking in `computeUnRotMatrix`'s degrees-in contract (`docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md` §4.5 item 1) |
| `test/test_stitching_session.py` | Create | Unit tests for `StitchingSession.claim_stitch()`/`finish_stitch()` atomicity, instantiated directly (no FastAPI TestClient) |
| `service.py` | Modify | Import the new module; `StitchingSession` gains filters + accepted-image tracking + atomic claim/finish; `SessionFolderHandler._process_new_file` gates new images; `run_stitching()` uses the accepted list + `build_data_matrix()`; `/session/{id}/stitch` and `/session/{id}/status` updated to match |
| `requirements.txt` | Modify | Add `piexif` (test-fixture generation only) |

---

## Task 1: `extract_flight_metadata()` — correct EXIF reader

**Files:**
- Create: `src/flight_metadata.py`
- Test: `test/test_flight_metadata.py`
- Modify: `requirements.txt`

**Interfaces:**
- Produces: `extract_flight_metadata(img_path: str) -> dict` with keys `latitude: float`, `longitude: float`, `altitude: float`, `roll: float` (degrees), `pitch: float` (degrees), `yaw: float` (degrees), `has_telemetry: bool`.

- [ ] **Step 1: Add `piexif` to `requirements.txt`**

Open `requirements.txt` and add this line right after the existing `exifread>=3.5.1` line:

```
piexif>=1.1.3          # For generating test-fixture EXIF (mirrors PROGRAM_JUANG sender.py's embedding)
```

Install it:

```bash
pip install piexif
```

- [ ] **Step 2: Write the failing test**

Create `test/test_flight_metadata.py`:

```python
import io
import json
import os
import sys
import unittest

import piexif

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.flight_metadata import extract_flight_metadata


def _make_test_jpeg_with_exif(path, lat, lon, alt_m, roll_deg, pitch_deg, yaw_deg):
    """Mirrors PROGRAM_JUANG/PROGRAM-SENDER1/sender.py's embed_flight_metadata_exif()."""
    import numpy as np
    import cv2

    blank = (np.zeros((10, 10, 3)) + 128).astype("uint8")
    ok, buf = cv2.imencode(".jpg", blank)
    jpeg_bytes = buf.tobytes()

    def _deg_to_dms_rational(deg_float):
        deg_float = abs(deg_float)
        d = int(deg_float)
        m_float = (deg_float - d) * 60
        m = int(m_float)
        s = (m_float - m) * 60
        return ((d, 1), (m, 1), (int(round(s * 100)), 100))

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
    attitude_json = json.dumps({"roll": round(roll_deg, 2), "pitch": round(pitch_deg, 2), "yaw": round(yaw_deg, 2)})
    zeroth_ifd = {piexif.ImageIFD.ImageDescription: attitude_json.encode("utf-8")}
    exif_bytes = piexif.dump({"GPS": gps_ifd, "0th": zeroth_ifd})

    out = io.BytesIO()
    piexif.insert(exif_bytes, jpeg_bytes, out)
    with open(path, "wb") as f:
        f.write(out.getvalue())


class TestExtractFlightMetadata(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = os.path.join(os.path.dirname(__file__), "_tmp_flight_metadata")
        os.makedirs(self.tmp_dir, exist_ok=True)

    def test_extracts_known_gps_and_attitude_in_degrees(self):
        path = os.path.join(self.tmp_dir, "known_pose.jpg")
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

    def test_missing_exif_returns_zero_pose_not_fabricated_gps(self):
        import numpy as np
        import cv2

        path = os.path.join(self.tmp_dir, "no_exif.jpg")
        blank = (np.zeros((10, 10, 3)) + 128).astype("uint8")
        cv2.imwrite(path, blank)

        result = extract_flight_metadata(path)

        self.assertFalse(result["has_telemetry"])
        self.assertEqual(result["latitude"], 0.0)
        self.assertEqual(result["longitude"], 0.0)
        self.assertEqual(result["roll"], 0.0)
        self.assertEqual(result["pitch"], 0.0)
        self.assertEqual(result["yaw"], 0.0)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd /home/abyan/Documents/orthomosaics/VISION-LIVESTITCH && python -m pytest test/test_flight_metadata.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'src.flight_metadata'`

- [ ] **Step 4: Write minimal implementation**

Create `src/flight_metadata.py`:

```python
# src/flight_metadata.py
"""
Correct EXIF/GPS/attitude extraction and admission filters for the
composable stitching orchestrator.

Unlike PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py's _extract_flight_metadata,
this module NEVER converts yaw/pitch/roll to radians -- geometry.computeUnRotMatrix
does that conversion itself (see docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md).
"""
import json
import math
from typing import Dict, List, Optional, Tuple

import exifread
import numpy as np


def extract_flight_metadata(img_path: str) -> Dict:
    """
    Read GPS (lat/lon/alt, GPS IFD) + yaw (GPSImgDirection) + roll/pitch
    (ImageDescription JSON) from a JPEG's EXIF, matching the convention
    PROGRAM_JUANG/PROGRAM-SENDER1/sender.py's embed_flight_metadata_exif()
    writes. All angles returned in DEGREES.

    Returns has_telemetry=False + zero pose (never fabricated GPS) if no
    GPS EXIF is present.
    """
    result = {
        "latitude": None, "longitude": None, "altitude": 0.0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "has_telemetry": False,
    }
    try:
        with open(img_path, "rb") as f:
            tags = exifread.process_file(f, details=False)

        if "GPS GPSLatitude" in tags and "GPS GPSLongitude" in tags:
            lat_vals = tags["GPS GPSLatitude"].values
            lon_vals = tags["GPS GPSLongitude"].values
            lat_ref = str(tags.get("GPS GPSLatitudeRef", "N"))
            lon_ref = str(tags.get("GPS GPSLongitudeRef", "E"))
            alt_tag = tags.get("GPS GPSAltitude")
            dir_tag = tags.get("GPS GPSImgDirection")

            lat = (lat_vals[0].num / lat_vals[0].den +
                   lat_vals[1].num / (lat_vals[1].den * 60) +
                   lat_vals[2].num / (lat_vals[2].den * 3600))
            lon = (lon_vals[0].num / lon_vals[0].den +
                   lon_vals[1].num / (lon_vals[1].den * 60) +
                   lon_vals[2].num / (lon_vals[2].den * 3600))
            alt = float(alt_tag.values[0].num / alt_tag.values[0].den) if alt_tag else 0.0
            yaw = float(dir_tag.values[0].num / dir_tag.values[0].den) if dir_tag else 0.0

            if lat_ref.strip() == "S":
                lat = -lat
            if lon_ref.strip() == "W":
                lon = -lon

            result.update({
                "latitude": lat, "longitude": lon, "altitude": alt,
                "yaw": yaw, "has_telemetry": True,
            })

        desc_tag = tags.get("Image ImageDescription")
        if desc_tag is not None:
            try:
                attitude = json.loads(str(desc_tag))
                result["roll"] = float(attitude.get("roll", 0.0))
                result["pitch"] = float(attitude.get("pitch", 0.0))
            except (ValueError, TypeError):
                pass

    except Exception:
        pass

    if result["latitude"] is None:
        result["latitude"] = 0.0
        result["longitude"] = 0.0
        result["altitude"] = 0.0
        result["roll"] = 0.0
        result["pitch"] = 0.0
        result["yaw"] = 0.0
        result["has_telemetry"] = False

    return result
```

- [ ] **Step 5: Run test to verify it passes**

Run: `python -m pytest test/test_flight_metadata.py -v`
Expected: `test_extracts_known_gps_and_attitude_in_degrees` and `test_missing_exif_returns_zero_pose_not_fabricated_gps` both PASS

- [ ] **Step 6: Commit**

```bash
git add src/flight_metadata.py test/test_flight_metadata.py requirements.txt
git commit -m "feat: add extract_flight_metadata with correct degrees-only EXIF parsing"
```

---

## Task 2: `build_data_matrix()` — GPS-to-local-meters + pose matrix

**Files:**
- Modify: `src/flight_metadata.py`
- Modify: `test/test_flight_metadata.py`

**Interfaces:**
- Consumes: metadata dicts shaped like `extract_flight_metadata()`'s return value (keys `latitude`, `longitude`, `altitude`, `yaw`, `pitch`, `roll`).
- Produces: `build_data_matrix(metadata_list: List[Dict], origin: Optional[Tuple[float, float]] = None) -> np.ndarray`, shape `(N, 6)`, columns `[X, Y, Z, Yaw, Pitch, Roll]`, angles in degrees — this is exactly the `dataMatrix` shape `Combiner.Combiner(images, dataMatrix, output)` expects.

- [ ] **Step 1: Write the failing test**

Add to `test/test_flight_metadata.py` (add this import near the top, alongside the existing one):

```python
from src.flight_metadata import extract_flight_metadata, build_data_matrix
```

Add this test class at the bottom, before `if __name__ == '__main__':`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_flight_metadata.py::TestBuildDataMatrix -v`
Expected: FAIL with `ImportError: cannot import name 'build_data_matrix'`

- [ ] **Step 3: Write minimal implementation**

Append to `src/flight_metadata.py` (after `extract_flight_metadata`):

```python
def build_data_matrix(metadata_list: List[Dict], origin: Optional[Tuple[float, float]] = None) -> np.ndarray:
    """
    Build the Nx6 [X, Y, Z, Yaw, Pitch, Roll] pose matrix Combiner.Combiner expects.
    X/Y are local meters relative to `origin` (defaults to the first entry's lat/lon).
    Yaw/Pitch/Roll stay in DEGREES -- geometry.computeUnRotMatrix converts to
    radians itself; converting here would reintroduce the double-conversion bug.
    """
    if origin is None:
        origin_lat = metadata_list[0]["latitude"]
        origin_lon = metadata_list[0]["longitude"]
    else:
        origin_lat, origin_lon = origin

    data_matrix = np.zeros((len(metadata_list), 6), dtype=np.float64)
    for i, meta in enumerate(metadata_list):
        lat = meta["latitude"]
        lon = meta["longitude"]
        x = (lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
        y = (lat - origin_lat) * 110540
        data_matrix[i, 0] = x
        data_matrix[i, 1] = y
        data_matrix[i, 2] = meta["altitude"]
        data_matrix[i, 3] = meta["yaw"]
        data_matrix[i, 4] = meta["pitch"]
        data_matrix[i, 5] = meta["roll"]
    return data_matrix
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_flight_metadata.py -v`
Expected: all tests PASS (5 total: 2 from Task 1, 1 new here — actually just added 1 new test method, so 3 total so far)

- [ ] **Step 5: Commit**

```bash
git add src/flight_metadata.py test/test_flight_metadata.py
git commit -m "feat: add build_data_matrix, consolidating duplicated GPS-to-meters math"
```

---

## Task 3: `GPSDistanceFilter`

**Files:**
- Modify: `src/flight_metadata.py`
- Modify: `test/test_flight_metadata.py`

**Interfaces:**
- Produces: `GPSDistanceFilter(threshold_m: float = 3.0)` with method `.should_accept(lat: float, lon: float) -> Tuple[bool, float]` (returns `(accepted, distance_from_last_accepted_m)`). Stateful per-instance (`_last_lat`, `_last_lon`) — Task 7 depends on one instance per `StitchingSession`, never shared globally.

- [ ] **Step 1: Write the failing test**

Add to `test/test_flight_metadata.py` imports:

```python
from src.flight_metadata import (
    extract_flight_metadata, build_data_matrix, GPSDistanceFilter,
)
```

Add test class:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_flight_metadata.py::TestGPSDistanceFilter -v`
Expected: FAIL with `ImportError: cannot import name 'GPSDistanceFilter'`

- [ ] **Step 3: Write minimal implementation**

Append to `src/flight_metadata.py`:

```python
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two lat/lon points."""
    R = 6371000  # Earth radius, meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class GPSDistanceFilter:
    """
    Redundancy filter: only accepts a new position if it's at least
    `threshold_m` away from the last ACCEPTED position. One instance
    per session -- sharing an instance across sessions/UAVs would compare
    positions from unrelated flights.
    """

    def __init__(self, threshold_m: float = 3.0):
        self.threshold_m = threshold_m
        self._last_lat: Optional[float] = None
        self._last_lon: Optional[float] = None

    def should_accept(self, lat: float, lon: float) -> Tuple[bool, float]:
        if self._last_lat is None:
            self._last_lat = lat
            self._last_lon = lon
            return True, 0.0

        dist = haversine_m(self._last_lat, self._last_lon, lat, lon)
        if dist >= self.threshold_m:
            self._last_lat = lat
            self._last_lon = lon
            return True, dist
        return False, dist
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_flight_metadata.py -v`
Expected: all tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/flight_metadata.py test/test_flight_metadata.py
git commit -m "feat: add GPSDistanceFilter for per-session redundancy gating"
```

---

## Task 4: `AttitudeThresholdFilter`

**Files:**
- Modify: `src/flight_metadata.py`
- Modify: `test/test_flight_metadata.py`

**Interfaces:**
- Produces: `AttitudeThresholdFilter(threshold_deg: float = 30.0)` with method `.should_accept(roll_deg: float, pitch_deg: float) -> Tuple[bool, float]` (returns `(accepted, worst_angle_deg)`).

- [ ] **Step 1: Write the failing test**

Add to imports:

```python
from src.flight_metadata import (
    extract_flight_metadata, build_data_matrix, GPSDistanceFilter,
    AttitudeThresholdFilter,
)
```

Add test class:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_flight_metadata.py::TestAttitudeThresholdFilter -v`
Expected: FAIL with `ImportError: cannot import name 'AttitudeThresholdFilter'`

- [ ] **Step 3: Write minimal implementation**

Append to `src/flight_metadata.py`:

```python
class AttitudeThresholdFilter:
    """
    Quality gate: rejects a frame if roll or pitch exceeds threshold_deg.
    Checked BEFORE GPSDistanceFilter in the admission pipeline -- swapping
    the order would let a tilt-rejected frame still mutate the GPS filter's
    reference position as a side effect.
    """

    def __init__(self, threshold_deg: float = 30.0):
        self.threshold_deg = threshold_deg

    def should_accept(self, roll_deg: float, pitch_deg: float) -> Tuple[bool, float]:
        worst = max(abs(roll_deg), abs(pitch_deg))
        return worst < self.threshold_deg, worst
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_flight_metadata.py -v`
Expected: all tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/flight_metadata.py test/test_flight_metadata.py
git commit -m "feat: add AttitudeThresholdFilter for per-session tilt gating"
```

---

## Task 5: Regression test locking in `computeUnRotMatrix`'s degrees contract

**Files:**
- Create: `test/test_geometry_unrot.py`

This does NOT modify `src/geometry.py` — it's a regression test proving the function's existing contract (degrees in, per `docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md` §4.5 item 1), so a future accidental change to that contract gets caught immediately, and so Task 1-4's degrees-only extraction can be trusted against it.

**Interfaces:**
- Consumes: `src.geometry.computeUnRotMatrix(pose: np.ndarray) -> np.ndarray` (existing, unmodified).

- [ ] **Step 1: Write the test**

Create `test/test_geometry_unrot.py`:

```python
import os
import sys
import unittest

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from src.geometry import computeUnRotMatrix


class TestComputeUnRotMatrixDegreesContract(unittest.TestCase):
    def test_zero_pose_returns_identity(self):
        pose = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        matrix = computeUnRotMatrix(pose)
        np.testing.assert_allclose(matrix, np.eye(3), atol=1e-9)

    def test_15_degree_pitch_produces_real_correction_not_57x_too_small(self):
        """
        This is the exact regression the double-conversion bug
        (docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md) would have broken:
        if pose[4] had already been converted to radians before reaching
        here, computeUnRotMatrix's own *pi/180 would shrink the real
        rotation by ~57x, and this matrix would come out nearly identity.
        """
        pose_zero = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        pose_tilted = np.array([0, 0, 0, 0, 15, 0], dtype=np.float64)  # 15 deg pitch

        matrix_zero = computeUnRotMatrix(pose_zero)
        matrix_tilted = computeUnRotMatrix(pose_tilted)

        # A real 15-degree correction must differ substantially from identity --
        # if the input were mistakenly pre-converted to radians (0.2618 passed
        # as "degrees"), the resulting matrix would be nearly indistinguishable
        # from matrix_zero. Assert the difference is NOT tiny.
        diff = np.abs(matrix_tilted - matrix_zero).max()
        self.assertGreater(diff, 0.05, "Correction is too small -- looks like the ~57x bug")


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run test to verify it passes against current (already-correct) `geometry.py`**

Run: `python -m pytest test/test_geometry_unrot.py -v`
Expected: both tests PASS (this locks in a currently-correct contract; it is not expected to fail, since `src/geometry.py` itself was never buggy — only the caller in `PROGRAM_JUANG` was)

- [ ] **Step 3: Commit**

```bash
git add test/test_geometry_unrot.py
git commit -m "test: lock in computeUnRotMatrix's degrees-in contract as a regression guard"
```

---

## Task 6: `StitchingSession` gains filters, accepted-image tracking, and atomic claim/finish

**Files:**
- Modify: `service.py:35-59` (the `StitchingSession` class)
- Create: `test/test_stitching_session.py`

**Interfaces:**
- Consumes: `GPSDistanceFilter`, `AttitudeThresholdFilter` from `src.flight_metadata` (Tasks 3-4).
- Produces: `StitchingSession.gps_filter`, `.attitude_filter`, `.accepted_images: List[Path]`, `.accepted_metadata: List[Dict]`, `.claim_stitch() -> bool`, `.finish_stitch() -> None` — Tasks 7-8 depend on these exact names.

- [ ] **Step 1: Write the failing test**

Create `test/test_stitching_session.py`:

```python
import os
import shutil
import sys
import unittest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from service import StitchingSession, StitchConfig


class TestStitchingSessionClaimStitch(unittest.TestCase):
    def setUp(self):
        self.test_root = os.path.join(os.path.dirname(__file__), "_tmp_session_test")
        os.chdir(os.path.dirname(os.path.dirname(__file__)))  # repo root, service.py uses relative "./sessions"
        config = StitchConfig(sessionId="test_claim_session")
        self.session = StitchingSession("test_claim_session", config)

    def tearDown(self):
        shutil.rmtree("./sessions/test_claim_session", ignore_errors=True)

    def test_claim_stitch_succeeds_when_not_stitching(self):
        self.assertTrue(self.session.claim_stitch())
        self.assertTrue(self.session.is_stitching)

    def test_claim_stitch_fails_when_already_stitching(self):
        self.session.claim_stitch()
        self.assertFalse(self.session.claim_stitch())

    def test_finish_stitch_allows_reclaiming(self):
        self.session.claim_stitch()
        self.session.finish_stitch()
        self.assertTrue(self.session.claim_stitch())

    def test_claim_stitch_records_accepted_count_at_claim_time(self):
        self.session.accepted_images = ["a.jpg", "b.jpg", "c.jpg"]
        self.session.claim_stitch()
        self.assertEqual(self.session.last_stitch_count, 3)

    def test_filters_are_independent_per_session_instance(self):
        config2 = StitchConfig(sessionId="test_claim_session_2")
        session2 = StitchingSession("test_claim_session_2", config2)
        try:
            self.session.gps_filter.should_accept(-7.25, 112.77)
            # session2's filter must be untouched by session1's call
            accepted, dist = session2.gps_filter.should_accept(10.0, 20.0)
            self.assertTrue(accepted)
            self.assertEqual(dist, 0.0)
        finally:
            shutil.rmtree("./sessions/test_claim_session_2", ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest test/test_stitching_session.py -v`
Expected: FAIL with `AttributeError: 'StitchingSession' object has no attribute 'claim_stitch'` (or similar for `gps_filter`/`accepted_images`)

- [ ] **Step 3: Modify `service.py`**

In `service.py`, add this import near the top, right after `from src import Combiner` (currently line 20):

```python
from src.flight_metadata import (
    extract_flight_metadata,
    build_data_matrix,
    GPSDistanceFilter,
    AttitudeThresholdFilter,
)
```

Replace the entire `StitchingSession` class (currently `service.py:35-59`) with:

```python
class StitchingSession:
    def __init__(self, session_id: str, config: StitchConfig):
        self.session_id = session_id
        self.config = config
        
        self.image_folder = Path(f"./sessions/{session_id}/images")
        self.image_folder.mkdir(parents=True, exist_ok=True)
        
        self.output_folder = Path(f"./sessions/{session_id}/output")
        self.output_folder.mkdir(parents=True, exist_ok=True)
        
        # Count existing images
        self.image_count = self._count_images()
        self.is_stitching = False
        self.should_stop = False
        self.ws_clients = []
        self.last_stitch_count = 0
        self.observer = None

        # Per-session filters -- NEVER share these across sessions (multi-UAV
        # sessions must not compare positions from unrelated flights)
        self.gps_filter = GPSDistanceFilter()
        self.attitude_filter = AttitudeThresholdFilter()
        self.accepted_images: List[Path] = []
        self.accepted_metadata: List[Dict] = []
        self._state_lock = threading.Lock()
    
    def _count_images(self):
        """Count all image files in the images folder"""
        count = 0
        for ext in ['*.jpg', '*.JPG', '*.jpeg', '*.JPEG', '*.png', '*.PNG', '*.tif', '*.TIF']:
            count += len(list(self.image_folder.glob(ext)))
        return count

    def claim_stitch(self) -> bool:
        """Atomically claim the stitching slot. Returns False if a stitch is already running."""
        with self._state_lock:
            if self.is_stitching:
                return False
            self.is_stitching = True
            self.last_stitch_count = len(self.accepted_images)
            return True

    def finish_stitch(self):
        with self._state_lock:
            self.is_stitching = False
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest test/test_stitching_session.py -v`
Expected: all 5 tests PASS

- [ ] **Step 5: Commit**

```bash
git add service.py test/test_stitching_session.py
git commit -m "feat: give StitchingSession per-session filters and atomic claim/finish_stitch"
```

---

## Task 7: Gate new-file admission in `SessionFolderHandler._process_new_file`

**Files:**
- Modify: `service.py:119-146` (`SessionFolderHandler._process_new_file`)

**Interfaces:**
- Consumes: `extract_flight_metadata()` (Task 1), `session.attitude_filter`/`session.gps_filter`/`session.accepted_images`/`session.accepted_metadata`/`session.claim_stitch()` (Task 6).

This task has no isolated unit test — `_process_new_file` is invoked by the watchdog observer thread and is exercised end-to-end in Task 9's manual pipeline verification. Apply the change directly and verify via Task 9.

- [ ] **Step 1: Replace `_process_new_file`**

Replace the entire method (currently `service.py:119-146`) with:

```python
    def _process_new_file(self, file_path):
        """Process a newly detected file: extract metadata, gate it, admit it if it passes."""
        if self.session_id not in sessions:
            return

        session = sessions[self.session_id]
        session.image_count = session._count_images()

        meta = extract_flight_metadata(file_path)

        if meta["has_telemetry"]:
            att_ok, worst_angle = session.attitude_filter.should_accept(meta["roll"], meta["pitch"])
            if not att_ok:
                print(f"[FILTER] Rejected (attitude {worst_angle:.1f}° over threshold): {file_path}")
                return

            gps_ok, dist = session.gps_filter.should_accept(meta["latitude"], meta["longitude"])
            if not gps_ok:
                print(f"[FILTER] Rejected (too close, {dist:.1f}m from last accepted): {file_path}")
                return
        else:
            print(f"[FILTER] Accepted WITHOUT telemetry (degraded confidence): {file_path}")

        session.accepted_images.append(Path(file_path))
        session.accepted_metadata.append(meta)

        print(f"[WATCHDOG] Session {self.session_id}: {len(session.accepted_images)} accepted images")

        # Notify clients
        asyncio.run(self._notify_clients(session, file_path))

        # Check if auto-stitch should trigger
        if session.config.auto_stitch_enabled:
            images_since_last_stitch = len(session.accepted_images) - session.last_stitch_count

            if images_since_last_stitch >= session.config.auto_stitch_threshold and session.claim_stitch():
                print(f"[AUTO-STITCH] Triggering for session {self.session_id} ({images_since_last_stitch} new accepted images)")

                # Trigger stitching
                threading.Thread(
                    target=lambda: asyncio.run(run_stitching(session.session_id, claimed=True)),
                    daemon=True
                ).start()
```

- [ ] **Step 2: Commit**

```bash
git add service.py
git commit -m "feat: gate new-file admission through attitude then GPS-distance filters"
```

---

## Task 8: `run_stitching()` uses the accepted list + `build_data_matrix()`, and endpoints match

**Files:**
- Modify: `service.py:238-326` (`run_stitching`)
- Modify: `service.py:452-464` (`trigger_stitch` endpoint)
- Modify: `service.py:502-517` (`get_session_status` endpoint)

**Interfaces:**
- Consumes: `build_data_matrix()` (Task 2), `session.accepted_images`/`accepted_metadata`/`claim_stitch()`/`finish_stitch()` (Task 6).

No isolated unit test for this task (it's the async/FastAPI glue) — exercised end-to-end in Task 9.

- [ ] **Step 1: Replace `run_stitching`**

Replace the entire function (currently `service.py:238-326`) with:

```python
async def run_stitching(session_id: str, claimed: bool = False):
    """Run the stitching process using the accepted (filtered) image list."""
    session = sessions[session_id]
    if not claimed and not session.claim_stitch():
        return

    # Notify clients that stitching started
    for ws in session.ws_clients:
        try:
            await ws.send_json({
                "type": "stitching_started",
                "image_count": len(session.accepted_images)
            })
        except:
            pass

    start_time = time.time()
    success = False
    error_msg = None

    try:
        images_to_stitch = list(session.accepted_images)
        metadata_to_stitch = list(session.accepted_metadata)

        print(f"[STITCH] Loading {len(images_to_stitch)} accepted images from {session.image_folder}")

        images = []
        valid_metadata = []
        for path, meta in zip(images_to_stitch, metadata_to_stitch):
            img = cv2.imread(str(path))
            if img is not None:
                images.append(img)
                valid_metadata.append(meta)
            else:
                print(f"[STITCH] Warning: failed to read {path}")

        if len(images) < 2:
            raise Exception("Not enough valid accepted images for stitching")

        dataMatrix = build_data_matrix(valid_metadata)

        print("[STITCH] Starting mosaic generation...")
        my_combiner = Combiner.Combiner(images, dataMatrix, str(session.output_folder))
        result = my_combiner.create_mosaic()

        if result is not None:
            output_path = session.output_folder / session.config.output_name
            cv2.imwrite(str(output_path), result)
            print(f"[STITCH] Saved result to {output_path}")
            success = True
        else:
            error_msg = "Stitching failed"

    except Exception as e:
        import traceback
        print(f"[STITCH] Error: {e}")
        traceback.print_exc()
        error_msg = str(e)

    elapsed_time = time.time() - start_time

    # Notify clients that stitching completed
    for ws in session.ws_clients:
        try:
            await ws.send_json({
                "type": "stitching_completed",
                "success": success,
                "elapsed_time": elapsed_time,
                "error_message": error_msg,
                "output_file": f"/session/{session_id}/result" if success else None
            })
        except:
            pass

    session.finish_stitch()
```

- [ ] **Step 2: Replace the `trigger_stitch` endpoint**

Replace the endpoint (currently `service.py:452-464`) with:

```python
@app.post("/session/{session_id}/stitch")
async def trigger_stitch(session_id: str, background_tasks: BackgroundTasks):
    if session_id not in sessions:
        return {"status": "Session not found"}

    session = sessions[session_id]

    if not session.claim_stitch():
        return {"status": "Stitching already in progress"}

    background_tasks.add_task(run_stitching, session_id, True)
    return {"status": "Stitching started", "image_count": len(session.accepted_images)}
```

- [ ] **Step 3: Replace the `get_session_status` endpoint**

Replace the endpoint (currently `service.py:502-517`) with:

```python
@app.get("/session/{session_id}/status")
async def get_session_status(session_id: str):
    if session_id not in sessions:
        return {"status": "Session not found"}
    
    session = sessions[session_id]
    return {
        "session_id": session_id,
        "image_count": session.image_count,
        "accepted_count": len(session.accepted_images),
        "is_stitching": session.is_stitching,
        "auto_stitch_enabled": session.config.auto_stitch_enabled,
        "auto_stitch_threshold": session.config.auto_stitch_threshold,
        "folder_monitoring_enabled": session.config.folder_monitoring_enabled,
        "last_stitch_count": session.last_stitch_count,
        "images_since_last_stitch": len(session.accepted_images) - session.last_stitch_count
    }
```

- [ ] **Step 4: Commit**

```bash
git add service.py
git commit -m "feat: run_stitching consumes accepted list via build_data_matrix; sync endpoints"
```

---

## Task 9: Full pipeline manual verification

**Files:** None modified — this is a manual acceptance check exercising Tasks 1-8 together.

- [ ] **Step 1: Confirm `piexif` is installed**

```bash
pip show piexif
```

Expected: shows version `1.1.3` or newer (installed in Task 1).

- [ ] **Step 2: Place the colleague's real farmland dataset**

A placeholder directory has already been created at:

```
dataset/colleague_farmland_flight/
```

Copy the image set your colleague used to generate the diagonal-skewed orthomosaic (the one shown earlier this session, with real embedded GPS+attitude EXIF from `PROGRAM_JUANG/PROGRAM-SENDER1/sender.py`) into that directory.

- [ ] **Step 3: Start the receiver**

```bash
cd /home/abyan/Documents/orthomosaics/VISION-LIVESTITCH
python receiver_socket.py
```

Expected output: `[*] Listening UAV1 pada port 5001` (and `UAV2` on 5002).

- [ ] **Step 4: Start `service.py`**

In a second terminal:

```bash
cd /home/abyan/Documents/orthomosaics/VISION-LIVESTITCH
python service.py
```

Expected output: `[STARTUP] Live Orthomosaic Stitcher starting...`, server listening on port 8001.

- [ ] **Step 5: Create a session with monitoring + auto-stitch enabled**

In a third terminal:

```bash
curl -X POST http://127.0.0.1:8001/session/create \
  -H "Content-Type: application/json" \
  -d '{"sessionId": "uav_1", "auto_stitch_threshold": 5, "auto_stitch_enabled": true, "folder_monitoring_enabled": true}'
```

Expected: `{"status":"Session created","session_id":"uav_1",...}`

- [ ] **Step 6: Send the dataset**

```bash
python sender_socket_sim.py --dataset-dir dataset/colleague_farmland_flight --port 5001 --delay 0.2
```

Expected: `[SENDER] ✓ Complete!` after all images are sent.

- [ ] **Step 7: Watch `service.py`'s terminal output**

Expected to see, interleaved:
- `[WATCHDOG] Session uav_1: N accepted images` incrementing (not necessarily 1:1 with files received, since some may be rejected)
- Possibly some `[FILTER] Rejected (...)` lines if any frames are too close together or too tilted
- `[AUTO-STITCH] Triggering for session uav_1 (...)` once the accepted threshold is crossed
- `[STITCH] Starting mosaic generation...` followed by `[STITCH] Saved result to sessions/uav_1/output/finalResult.png`

- [ ] **Step 8: Fetch and visually compare the result**

```bash
curl -o /tmp/new_result.png http://127.0.0.1:8001/session/uav_1/result
```

Open `/tmp/new_result.png` and compare against the earlier diagonal-skewed orthomosaic (`docs/WhatsApp Image 2026-09-07 at 13.32.51.jpeg`).

**Expected outcome** (per the analysis in `docs/DRIFT_MISREGISTRATION.md` and `docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md`, confirmed with the user earlier this session): the whole-mosaic rotational skew should be visibly reduced or gone, since real attitude is now correctly reaching `computeUnRotMatrix` in degrees. The separate single-frame ghosting/mismatch artifact (the red-line match anomaly) is expected to still be present -- its root cause is in `Combiner.py`'s matching logic (`docs/DRIFT_MISREGISTRATION.md` §9 checklist), which is explicitly out of scope for this plan. **This partial result is success, not failure** -- do not attempt to fix the ghosting artifact as part of this plan.

- [ ] **Step 9: Verify the fallback path with a no-EXIF image**

```bash
python3 -c "
import cv2
import numpy as np
blank = (np.zeros((100, 100, 3)) + 128).astype('uint8')
cv2.imwrite('/tmp/no_exif_test.jpg', blank)
"
cp /tmp/no_exif_test.jpg dataset/colleague_farmland_flight/zz_no_exif_test.jpg
python sender_socket_sim.py --dataset-dir dataset/colleague_farmland_flight --port 5001 --delay 0.2
```

Expected in `service.py`'s terminal: a line reading `[FILTER] Accepted WITHOUT telemetry (degraded confidence): .../zz_no_exif_test.jpg` -- confirming the image was accepted (not silently dropped, not given fabricated GPS) and clearly flagged.

- [ ] **Step 10: Clean up test artifacts**

```bash
rm -f dataset/colleague_farmland_flight/zz_no_exif_test.jpg /tmp/no_exif_test.jpg /tmp/new_result.png
```

---

## Self-Review Notes

- **Spec coverage**: all 4 module pieces (Tasks 1-4), the `computeUnRotMatrix` regression guard (Task 5), `StitchingSession` wiring (Task 6), admission gating (Task 7), stitch execution + endpoints (Task 8), and the full agreed testing plan items 1-5 from `docs/COMPOSABLE_ORCHESTRATOR_DESIGN.md` §5 (Tasks 1/9, 5, 3/4, 9, 1) are all covered.
- **Placeholder scan**: no TBD/TODO; the one user-supplied-data step (Task 9, Step 2) is an explicit manual instruction, not a hardcoded path a test silently depends on.
- **Type consistency**: `extract_flight_metadata` return dict keys (`latitude`, `longitude`, `altitude`, `roll`, `pitch`, `yaw`, `has_telemetry`) are used identically across Tasks 1, 2, 6, 7. `GPSDistanceFilter`/`AttitudeThresholdFilter` constructor and `.should_accept()` signatures are used identically across Tasks 3/4/6/7. `claim_stitch()`/`finish_stitch()` names match between Task 6's definition and Tasks 7/8's call sites.
