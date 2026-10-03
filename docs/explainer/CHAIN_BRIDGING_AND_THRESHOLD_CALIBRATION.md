# Chain-Bridging Architecture & Sanity-Check Threshold Calibration

> **Status**: Checkpoint reached. Current config is stable and validated against a known-good baseline. Next planned work: overlap density (`GPS_DISTANCE_THRESHOLD_M`), tracked separately.
> **Scope**: `src/Combiner.py` — continuation of `docs/GPS_SANITY_CHECK_DEBUG_LOG.md`, picking up after the "revert threshold experiment / implement spatial-spread check" checkpoint.
> **Dataset**: same real flight (2026-09-28, `uav_1`, 61 raw images, content is fixed — replays get new filename timestamps but identical image content).

---

## 1. Problem Restated

After the GPS translation sanity check + inlier ratio gate were implemented and debugged (see the prior doc), the mosaic was structurally correct (no catastrophic kink) but suffered from a **new, distinct problem**: visible ghosting, soft double-edges, and "staircase" seams — most noticeably at a T-junction near a warehouse/silo, and along the corridor road. This was **not** the same failure mode as the original farmland-aliasing kink; it was a *density/positioning* artifact that appeared specifically once quality gates started rejecting frames.

The user's baseline for comparison throughout this investigation was `docs/dump/BestCombiner.py` (an earlier, simpler version of the engine, still using a physics-based/uncalibrated sanity check) — specifically its `intermediateResult_44.png`, produced with sanity check bypassed and only a loose inlier-ratio gate active. That baseline is visibly crisp: sharp road, no ghosting, no staircase. Matching that crispness while keeping real protection against misregistration was the goal of this session.

---

## 2. Root Cause: The Chain-Bridging Gap

`Combiner.combine(index)` chains position via:
```python
H_global_current = np.dot(self.H_global_prev, H_rel_3x3)
```
`H_rel_3x3` is the transform between `image_list[index-1]` and `image_list[index]` — computed **unconditionally on raw array position**, regardless of whether `index-1` itself was accepted or rejected by any gate. `self.H_global_prev`, however, only advances on the **accepted-frame path** (inside the blend section). When a frame is rejected, the function simply does `return self.result_image` — `H_global_prev` stays frozen at wherever it was after the last successfully blended frame.

Consequence: if frame `N` is rejected, frame `N+1` still measures its transform relative to raw frame `N` (a valid local measurement), but that measurement gets chained onto `H_global_prev`, which represents frame `N-1`'s position, not `N`'s. The real motion from `N-1 → N` is implicitly treated as zero. This produces a visible position jump for frame `N+1` — worse, if the images at `N-1`/`N`/`N+1` are close together spatially (e.g. a building complex spanning several frames), several of these jumps compound into a "staircase" of near-duplicate, slightly offset slivers, and heavy feather-blending of those slivers reads as soft ghosting.

This bug exists in **both** `BestCombiner.py` and the current `src/Combiner.py` — it is not new. It stays invisible in `BestCombiner.py` only because that engine rejects rarely (~12% of pairs, isolated) with just one loose inlier-ratio gate. The current engine, with a working (calibrated) GPS sanity check added on top, rejects more often (~27-28%), so the same latent bug fires far more frequently and becomes visible.

---

## 3. Three Bridging Strategies Tried, All With Real Failure Modes

### 3.1 Always-bridge, translation-only fallback
On every rejection, compute a translation-only `H_rel` from the GPS delta (`meters_per_pixel`-calibrated, Y-axis flipped to match image row convention) and advance `H_global_prev` with it (no pixels painted). **Result**: ghosting spread throughout the mosaic, not just at rejection points — confirmed via instrumentation: this fallback fired **16 of 56** `combine()` calls (29%), including 6 times inside the otherwise-clean corridor section (not just the maneuver zone). Even a single use of a rotation-blind approximation introduces enough error to be visible once blended.

### 3.2 Always-bridge, yaw-aware fallback
Hypothesis: residual rotation between two independently-unrotated frames correlates with `-yaw_delta` (measured correlation **-0.73** across 44 real frame pairs; using `theta = -yaw_delta` directly reduced residual std from 2.27° to 1.58°, a real signal). Implemented rotation-about-image-center using this estimate. **Result**: side-by-side crop comparison (T-junction, bottom road, upper field) showed this was **not** visibly better than the translation-only version — all three regions still showed ghosting/staircase artifacts comparable to or worse than 3.1. The yaw-based rotation estimate, while statistically real, was not precise enough per-application to fix the visible blend quality.

### 3.3 Skip rejected frames entirely (match against `last_accepted_index`)
Instead of bridging with an approximation, track `self.last_accepted_index` and always match the *incoming* frame against the last frame that was actually accepted (not the raw `index-1`), so no gap ever needs bridging. **Result**: catastrophic. In a live test, `last_accepted_index` got stuck at index 8 after a rejection at index 9, and **never recovered for the remaining 46 frames** to the end of the flight. Root cause: once the reference frame falls behind, the real-world distance between it and the current frame keeps growing (the reference never advances while the drone keeps flying forward), which shrinks visual overlap and match count, which makes the *next* match even less likely to succeed — a runaway positive-feedback collapse. Confirmed via match-count degradation in the log (41 → 32 → 23 → ... → 0).

**Conclusion from all three**: there is no free lunch. Freezing on rejection produces position jumps; bridging (by any method tried) produces ghosting; skip-ahead risks total collapse. The type of artifact differs, but every rejection leaves *some* visible trace under any handling strategy.

---

## 4. Resolution: Bounded Fallback + Data-Driven Threshold Retuning

Given no bridging strategy is artifact-free, the approach taken was to **minimize how often bridging is even needed**, and cap how much bridging can compound when it is:

### 4.1 Architecture: reverted to plain `index-1` chaining
Dropped `last_accepted_index` entirely. `combine(index)` always matches against `image_list[index-1]`, matching `BestCombiner.py`'s original approach. All GPS/dataMatrix comparisons (`__check_gps_translation_sanity`, mpp calibration) likewise reference `index-1` directly — no more `ref_index` indirection.

### 4.2 Bounded bridging (`__bridge_or_freeze`)
```python
self._consecutive_rejections = 0   # __init__
self.MAX_BRIDGE_GAP = 3            # __init__

def __bridge_or_freeze(self, index, image2_shape):
    self._consecutive_rejections += 1
    if self._consecutive_rejections <= self.MAX_BRIDGE_GAP:
        fallback = self.__simple_fallback_transform(index)
        if fallback is not None:
            self.__advance_global_position(fallback, image2_shape, will_paint=False)
    # beyond MAX_BRIDGE_GAP: do nothing (freeze), rather than compound
    # approximations over a growing, uncertain gap
```
Called at all 5 rejection points in `combine()`. `self._consecutive_rejections` resets to `0` on the accepted-frame path. `__simple_fallback_transform` is the translation-only (no rotation) GPS fallback from §3.1 — the yaw-aware version was **not** kept, since §3.2 showed no measurable benefit over the simpler version, and simplicity was preferred absent proven gain.

`__simple_fallback_transform` guards against fabricating motion from frames with no real telemetry:
```python
prev_pos = self.dataMatrix[index - 1, 0:2]
curr_pos = self.dataMatrix[index, 0:2]
if np.all(prev_pos == 0) or np.all(curr_pos == 0):
    return None  # no telemetry -- freeze instead of guessing
```
**Known false-positive**: `dataMatrix[0]` (the very first frame) is *always* exactly `[0, 0]` by construction (`build_data_matrix` uses the first entry as the coordinate origin), regardless of whether that frame actually had real telemetry. This trips the zero-check for the very first rejected pair (`0-1`) even when frame 0's telemetry was perfectly valid. Low impact in practice (nothing has been placed yet at that point, so bridging wouldn't have mattered), but it is a real ambiguity in the check's logic worth fixing if the false trigger ever matters elsewhere.

### 4.3 A note on `__advance_global_position` itself
Directly diffed against `BestCombiner.py`'s inline chaining math (side by side) — **identical formula** for the accepted-frame path (`H_global_prev @ H_rel_3x3`, normalize, compute canvas bounds, re-base translation). The only difference is *where* it's called from: `BestCombiner.py` calls it once per frame (accept only); the current engine calls it up to twice per frame (accept, or bridge-on-reject via `will_paint=False`). The bug, such as it is, was never in the chaining math — it was in the existence of the extra reject-path call sites, which is exactly what bounded bridging is designed to minimize the impact of.

---

## 5. Two Gate Values Were Never Independent — And Should Not Be Equal

`combine()` uses inlier ratio in two places:
1. **Accept/reject gate** (line ~391): `if inlier_ratio < 0.6: reject`.
2. **Calibration-sample eligibility** (line ~405, inside the `meters_per_pixel` bootstrap block): `if inlier_ratio > X and gps_dist_m > 2.0: <count this frame as a calibration sample>`.

**Bug found**: whenever `X <= 0.6` (the accept threshold), condition #2 is **dead code** — by the time execution reaches it, the frame has already survived gate #1, so `inlier_ratio >= 0.6` is already guaranteed, and `inlier_ratio > X` is trivially true for any `X <= 0.6`. This was true from the start (originally `X = 0.50 < 0.6`) and remained true through an experiment that lowered *both* values together to `0.4`.

This matters because `meters_per_pixel` is calibrated **once** (first 5 eligible samples) and then frozen for the rest of the session — every subsequent GPS sanity check depends on it. A calibration-eligibility bar that is trivially satisfied contributes no actual quality filtering to that one-time, high-stakes calibration.

**Fix applied**: decoupled the two, with eligibility set *stricter* than acceptance (not equal, not looser): accept gate stays at `0.6`, eligibility raised to `0.7` (i.e., only frames good enough to trust for the one-time scale calibration, not merely good enough to blend, get to vote on it).

---

## 6. Threshold Calibration: Corridor Noise vs. Maneuver-Zone Kink

### 6.1 The idea
The GPS sanity check (`__check_gps_translation_sanity`) exists to catch exactly one thing: the drone's maneuver-zone banking turn, which produces geometrically implausible transforms (matching `docs/DRIFT_MISREGISTRATION.md`'s farmland-aliasing/misregistration hypothesis, just triggered by attitude dynamics rather than periodic texture). It should **not** fire on ordinary corridor frames whose translation error is just SIFT/GPS measurement noise. The original tolerance (`TRANSLATION_TOLERANCE_M = 4.5`) was a guess, not measured against real data, and was shown to reject several corridor pairs that were visually fine.

### 6.2 The measurement
For a real pipeline run (mpp calibrated to `0.357971 m/px`), `translation_error_px = norm(estimated_center_translation - GPS_implied_translation)` was computed for every pair that the (pre-retune) sanity check rejected, split into two groups by ground truth (confirmed via match visualizations and known maneuver-zone attitude data):

| Group | Pair | Error (px) | Error (m) |
|---|---|---|---|
| Corridor (should pass) | 27→28 | 13.77 | 4.93 |
| Corridor (should pass) | 36→37 | 16.33 | 5.84 |
| Corridor (should pass) | 19→20 | 17.88 | 6.40 |
| Corridor (should pass) | 33→34 | 21.21 | 7.59 |
| Corridor (should pass) | 39→40 | 21.75 | **7.79** |
| Maneuver (should reject) | 46→47 | 34.87 | **12.48** |
| Maneuver (should reject) | 43→44 | 43.65 | 15.62 |

There is a clean, empty gap between **7.79 m** (worst corridor case) and **12.48 m** (best-behaved maneuver case) — no observed pair falls inside it. Any tolerance value in that range cleanly separates the two populations using only real, already-collected data (no new guessing).

### 6.3 Value chosen
`TRANSLATION_TOLERANCE_M = 10.0` — the midpoint of the gap (`(7.79 + 12.48) / 2 ≈ 10.1`, rounded), giving roughly symmetric margin to both sides (+2.2 m over the corridor ceiling, -2.5 m under the maneuver floor).

Secondary consideration: even with a looser sanity-check tolerance, most maneuver-zone frames are independently caught by the inlier-ratio gate and the `len(matches) < 4` floor anyway, since real match counts collapse during the banking turn (observed as low as 4-7 matches) regardless of the sanity check.

### 6.4 Result
Direct crop comparison (T-junction, bottom road) between the retuned engine and `ini_september/output/intermediateResult_44.png` (the ungated `BestCombiner` baseline) showed the T-junction area now nearly indistinguishable from baseline — sharp roof edges, clean road, no visible doubling. Remaining minor seams along field boundaries in both images are attributed to overlap density (§7), not to sanity-check false rejections anymore.

---

## 7. Current Checkpoint — Config Values

```python
# src/Combiner.py
inlier_ratio < 0.6          # accept/reject gate (unchanged from original)
inlier_ratio > 0.7          # calibration-sample eligibility (raised from 0.5, decoupled from gate above)
TRANSLATION_TOLERANCE_M = 10.0   # GPS sanity check tolerance (raised from 4.5, data-driven)
MAX_BRIDGE_GAP = 3           # consecutive rejections still eligible for translation-only bridging
```
Admission filter: `GPSDistanceFilter(threshold_m=3.0)` in `service.py`, `AttitudeThresholdFilter()` default (30°).

No spatial cross-validation check (was implemented and removed — see `docs/GPS_SANITY_CHECK_DEBUG_LOG.md` for that investigation; superseded here by the simpler bounded-bridge + retuned-threshold combination, which achieved comparable visual quality with less architectural complexity).

---

## 8. Open Items / Next Steps

1. **Overlap density** (`GPS_DISTANCE_THRESHOLD_M`, currently `3.0`) — deliberately deferred. Now that the sanity check false-reject problem is resolved, it may be safe to lower this again (more admitted frames, denser overlap, less feather-blend ghosting) without reintroducing the maneuver-zone kink risk, since the sanity check is now correctly calibrated to catch it. Not yet tested.
2. The `dataMatrix[0] == [0,0]`-by-construction false positive in the telemetry-zero guard (§4.2) — low impact today, worth a real fix (e.g. check `has_telemetry` directly instead of inferring from position) if it starts mattering.
3. Yaw-aware fallback rotation (§3.2) was abandoned for lack of proven benefit — not ruled out permanently, just not worth the added complexity given current results.
4. **Cross-camera threshold portability — lens undistortion** (§9, new finding below). High priority, not yet started.

---

## 9. New Finding: Thresholds Don't Transfer Across Camera Models — Likely Root Cause is Uncorrected Lens Distortion

### 9.1 What happened
Ran the same pipeline (identical code, identical tuned thresholds from §7) against a completely different dataset: `dataset/test_2`, captured with a **GoPro** (wide/fisheye-style lens) instead of the DJI Osmo Action 5 Pro used for all prior tuning. Two problems surfaced:

- **Much slower**: raw resolution `4872x5568` vs the DJI dataset's `1080x1920` (~15x more pixels). After the same `/5` downsample, working images are `~1113x974` vs `~280-420px` on the DJI dataset. SIFT feature detection cost scales with pixel count, so per-frame feature detection went from ~0.02-0.03s (DJI) to ~0.17-0.2s (GoPro) — matches the pixel-count ratio reasonably well.
- **Much higher rejection rate, but almost entirely via `inlier_ratio < 0.6`**, not the GPS sanity check. Match counts were actually *huge* (200-800 good matches vs DJI's typical 20-100), but `inlier_ratio` stayed mostly in the 0.2-0.5 range, rarely crossing `0.6`. Because calibration eligibility requires `inlier_ratio > 0.7`, `meters_per_pixel` **never calibrated** for the entire run — the GPS sanity check never got a chance to run at all; `inlier_ratio` alone did all the rejecting.

Visual comparison against an ungated reference (`output/INI TERAKHIR/output/final_result_kali.png` — pure chained-homography + predictive ROI, no inlier/sanity gates) showed the *raw* per-frame registration is actually fine on this dataset (the ungated mosaic covers a much larger area and still looks clean, no dramatic kink). This means the DJI-tuned `inlier_ratio < 0.6` gate is rejecting many pairs on the GoPro dataset that are genuinely fine — the threshold itself, not the underlying registration quality, is the problem.

### 9.2 Working hypothesis (not yet confirmed)
`inlier_ratio` measures how well a matched point set fits `cv2.estimateAffinePartial2D`'s **similarity-transform model** (rotation + uniform scale + translation only — no lens distortion term). A wide-angle/fisheye lens has real barrel distortion; points far from the frame center genuinely don't obey a pure similarity transform even when the correspondence itself is correct, so RANSAC systematically flags more of them as "outliers." This would produce a structurally lower `inlier_ratio` for wide/fisheye cameras *regardless of registration quality* — explaining both the huge match counts (matching itself works fine) and the low inlier ratio (the *model*, not the data, is the limiting factor).

### 9.3 Which thresholds are camera-coupled vs. portable
- `TRANSLATION_TOLERANCE_M` (§6): likely **more portable** across cameras. It's expressed in meters, and `meters_per_pixel` is already auto-calibrated per-session from real GPS+SIFT data, so the tolerance represents a physical quantity (typical GPS/SIFT position noise vs. a genuine misregistration jump) rather than a camera-specific pixel behavior. Not yet validated on a second camera, but structurally less likely to need re-tuning.
- `inlier_ratio` threshold (§5 doc, `0.6`/`0.7`): likely **camera-coupled**, specifically by lens distortion characteristics, per the hypothesis above.

### 9.4 To-do: lens undistortion as the real fix (not per-dataset re-tuning)
The user's goal is a system that works across camera configurations with **minimal manual calibration** per deployment. Re-tuning `inlier_ratio` per dataset defeats that goal and only treats the symptom. The architecturally correct fix: **undistort images before feature detection**, using each camera *model's* intrinsic distortion coefficients (`cv2.undistort()` / `cv2.fisheye.undistortImage()`). This is fundamentally different in scope from the existing `meters_per_pixel` auto-calibration:

| | Scope | When calibrated | Already automatic? |
|---|---|---|---|
| `meters_per_pixel` | Per **flight session** (altitude/speed vary per mission) | Runtime, first 5 eligible frames | Yes |
| Lens distortion coefficients | Per **camera model** (fixed hardware property) | Once per camera model (e.g. standard OpenCV checkerboard calibration), stored as a config/profile | **No — not implemented** |

Once distortion is corrected upstream, images from any camera should approximately satisfy the similarity-transform assumption for a roughly-planar nadir shot, and `inlier_ratio` should behave consistently across camera models — potentially making the existing threshold values (or something close to them) genuinely portable, closing the gap toward the "plug in a new camera, minimal re-tuning" goal.

**Not yet started.** Would need: (a) a way to identify which camera/profile captured a given session (e.g. EXIF `Make`/`Model`), (b) a small library of per-camera-model distortion coefficients (calibrated once per camera, reusable across all future flights with that camera), (c) an undistortion step added to `__preprocess_images` before the downsample/unrotation steps already there.
