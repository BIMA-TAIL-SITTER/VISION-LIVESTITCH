# Bug Report: Degrees/Radians Double-Conversion Disables IMU Unrotation

> **Status**: Confirmed, not yet fixed.
> **Severity**: Silent correctness bug — no crash, no error, no warning. The feature appears to work (data flows end-to-end) but its actual effect is ~57x weaker than intended.
> **Found in**: `PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py` (colleague's fork), interacting with `src/geometry.py` (shared core, unmodified).
> **Found while**: Reading colleague's codebase to understand his fork of the stitching pipeline (fixed-wing UAV variant).

---

## 1. Summary

Your colleague's fork extends the stock pipeline to actually pass real flight attitude (yaw/pitch/roll) into the stitcher, instead of the stock behavior of always sending zeros (see [TECHNICAL.md §10.2](./TECHNICAL.md#102-image-only-mode-current-state)). This is a real improvement — the whole point of `computeUnRotMatrix` in `src/geometry.py` is to correct for camera tilt using exactly this data.

However, the value gets converted from degrees to radians **twice** before the rotation math runs on it — once explicitly in `stitcher.py`, and once again inside `computeUnRotMatrix` itself, which expects degrees and does its own conversion. The result: the correction applied is divided by roughly 57× (that's `180/π`) more than intended, so a genuine 15° tilt is treated as if it were about 0.26°. The unrotation step effectively does almost nothing, silently — no exception, no log warning, the pipeline "runs fine."

---

## 2. The Evidence, Line by Line

**`src/geometry.py:6-14`** (`computeUnRotMatrix`, part of the shared/unmodified core):

```python
def computeUnRotMatrix(pose):
    '''
    :param pose: A 1x6 NumPy ndArray containing pose information in [X,Y,Z,Y,P,R] format
    :return: A 3x3 rotation matrix that removes perspective distortion from the image to which it is applied.
    '''
    a = pose[3]*np.pi/180 # alpha
    b = pose[4]*np.pi/180 # beta
    g = pose[5]*np.pi/180 # gamma
```

The `*np.pi/180` is a degrees-to-radians conversion, applied by this function itself. That tells you unambiguously: **this function expects `pose[3:6]` to already be in degrees.**

**`PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py:337-353`** (`_stitch_external`, in `StitchingEngine`):

```python
# yaw/pitch/roll sekarang terisi dari metadata EXIF asli (sender.py),
# sebelumnya selalu 0 (belum ada sumber datanya).
# CATATAN: dikonversi ke radian karena itu konvensi umum untuk data
# orientasi kamera/pesawat — verifikasi ulang ke source Combiner
# apakah dia memang mengharapkan radian atau derajat di kolom ini.
yaw   = math.radians(gps.get("yaw", 0.0))
pitch = math.radians(gps.get("pitch", 0.0))
roll  = math.radians(gps.get("roll", 0.0))

x = (lon - origin_lon) * 111320 * math.cos(math.radians(origin_lat))
y = (lat - origin_lat) * 110540
data_matrix[i, 0] = x
data_matrix[i, 1] = y
data_matrix[i, 2] = alt
data_matrix[i, 3] = yaw
data_matrix[i, 4] = pitch
data_matrix[i, 5] = roll
```

The comment is worth reading closely — your colleague *noticed* this was ambiguous and flagged it for verification ("verify against the Combiner source whether it actually expects radians or degrees in this column"), then guessed radians. The guess was wrong: `computeUnRotMatrix` wants degrees, as shown above.

So the pipeline for, say, a 15° roll looks like:

```
sender.py / EXIF:          roll = 15.0            (degrees, correct so far)
stitcher.py:337-344:       roll = radians(15.0)   = 0.2618          (converted to radians — WRONG, too early)
data_matrix[i, 5] = 0.2618                          (stored as "0.2618", but computeUnRotMatrix will treat this AS IF it were 0.2618 degrees)
geometry.py:14:            g = 0.2618 * (pi/180)   = 0.00457 radians = 0.26°   (converted AGAIN, as if input was in degrees)
```

The real 15° tilt is treated as 0.26° — divided by ~57.3× (`180/π`, the conversion factor applied where it shouldn't be).

---

## 3. Why This Is Hard to Notice

- **No crash.** `data_matrix[i, 5]` is a valid float either way; `computeUnRotMatrix` runs to completion regardless of whether the number means "15 degrees" or "0.2618 (mis-scaled) degrees."
- **No error path triggered.** This isn't a `None`, a missing key, or an exception — it's a silently-wrong number flowing through code that has no way to know the caller's intended unit.
- **The IMU gate (onboard + ground, both in *degrees*) still works correctly** — `ROLL_THRESHOLD_DEG` / `PITCH_THRESHOLD_DEG` comparisons in `sender.py` and `AttitudeThresholdFilter` in `stitcher.py` both operate on the original degree values read straight from MAVLink/EXIF, untouched by this bug. So frames still get correctly held during hard turns — the gate logic and the stitching-math logic read the attitude value at different points in the pipeline, and only the second one has the unit bug. This is exactly why the system "looks like it's working" in day-to-day testing.
- **The output still looks like a mosaic.** Since the gates already reject anything above ~30-45°, the *residual* tilt on images that do make it through is usually modest (a few degrees). With unrotation effectively disabled, those images just don't get their small residual tilt corrected — the stitcher still produces *a* result, just with an accuracy characteristic close to having no unrotation step at all.

---

## 4. Impact

- **Direct**: any actual camera tilt within the gated range (< 30-45°, i.e. everything that makes it through both filters) is essentially **not corrected** by `computeUnRotMatrix`, because the angle it receives is ~57× smaller than reality. `InvR` ends up very close to the identity matrix regardless of the true tilt.
- **Indirect**: uncorrected tilt distorts frame-to-frame geometry beyond what pure translation/rotation can capture, which stacks on top of (but is a distinct root cause from) the SIFT descriptor aliasing issue in periodic farmland texture documented in [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md). Both bugs can degrade the same symptom (bad frame placement) through different mechanisms — worth keeping them separate when testing fixes, so a fix for one isn't mistakenly credited for a symptom actually caused by the other.
- **Scope**: this only affects `stitcher.py`'s path into `Combiner` (the fixed-wing fork). It does not affect `sender.py`'s IMU gate, EXIF embedding, or GPS threshold filtering — all of those read attitude independently and correctly, in degrees.

---

## 4.5. Scope Note: This Fix Is Fork-Local, Not Shared-Core

Unlike the issues tracked in [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md) (which all live in `src/Combiner.py`, imported unforked by both pipelines), this bug straddles a boundary:

- `src/geometry.py` (`computeUnRotMatrix`) — **shared core**, unmodified, contract confirmed correct (expects degrees, matches `src/ImageMosaic.py:71-73`'s own `# Yaw (degrees)` comments).
- `PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py:342-344` — **fork-local**, this is where the bad conversion actually happens.

Practical consequence: **testing purely inside `src/` will not exercise this bug at all.** Your own `service.py:285-287` hardcodes `Yaw = Pitch = Roll = 0` unconditionally — it never attempts to pass real attitude data through `src/Combiner` → `src/geometry.computeUnRotMatrix` in the first place, so running `service.py` (or `src/ImageMosaic.py` with a zeroed pose matrix) can't surface or validate this fix either way. To actually prove the fix, either:

1. Write a small standalone script that calls `src.geometry.computeUnRotMatrix(pose)` directly with a known degree value (e.g. `pose = [0,0,0, 0, 15, 0]` for 15° pitch) and confirm the returned matrix reflects that rotation — isolates the shared function's contract without touching either pipeline.
2. Apply the fix directly to `PROGRAM_JUANG/PROGRAM-SENDER1/stitcher.py` (Option A, §5) and re-run the verification steps in §6 against his actual pipeline.

---

## 5. The Fix

Two equally valid options — pick whichever matches how you want unit conventions to read across the codebase:

**Option A (minimal diff, recommended)**: stop converting to radians in `stitcher.py`, since `computeUnRotMatrix` already does that conversion itself.

```python
# stitcher.py, replace lines 342-344:
yaw   = gps.get("yaw", 0.0)
pitch = gps.get("pitch", 0.0)
roll  = gps.get("roll", 0.0)
```

**Option B**: keep `stitcher.py`'s radian conversion, and instead change `computeUnRotMatrix` in `src/geometry.py` to accept radians directly (drop the `*np.pi/180` on lines 12-14). Not recommended unless you're sure nothing else in the codebase calls `computeUnRotMatrix` expecting the current degrees-in contract — `src/ImageMosaic.py` (the batch CLI entry point) likely also calls this function with degree-based pose data, so changing the function's contract could silently break that caller instead. Option A only touches the one call site that has the bug.

---

## 6. Verification Steps

After applying Option A:

1. Run `imu_simulator.py --mode scenario --max-roll 30` (or a real flight) to produce frames with known, non-trivial roll.
2. Confirm via the EXIF check snippet in `README.md` (`## GPS + Attitude Metadata pada Citra (EXIF)` section) that the embedded roll/pitch match what the simulator/FC reported.
3. Add a temporary print/log of `data_matrix[i, 3:6]` right before `Combiner.Combiner(images, data_matrix, ...)` is constructed in `stitcher.py`, and confirm the values now match the degree magnitudes from EXIF (not divided by ~57×).
4. Run a short stitch and visually compare the mosaic against a pre-fix run on the same image set — look specifically at frames captured with non-trivial residual tilt (a few degrees, just under the gate threshold); those should show a visible difference in how well their edges align, since they're the ones the fix actually changes behavior for.

---

*Related reading: [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md) for the separate farmland/periodic-texture misregistration issue. [EXPLAINER.md §3.4](./EXPLAINER.md#34-slide-it-into-place-frame-after-frame--chained-matrix-multiplication) for the plain-language mental model of how `computeUnRotMatrix`'s output feeds into the rest of the pipeline.*
