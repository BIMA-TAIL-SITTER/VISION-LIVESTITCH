# Task 9 Verification Findings — Composable Orchestrator, Full Pipeline Test

> **Context**: Task 9 of `docs/superpowers/plans/2026-09-15-composable-orchestrator.md` — manual end-to-end verification of the attitude-extraction fix (`src/flight_metadata.py`) against the colleague's real farmland dataset. Run on 2026-09-26, session `uav_1`.
> **Status**: Task 9 complete. Findings below, plus recorded to-dos for follow-up work.

---

## 1. Run History (two attempts, only the second is a valid comparison)

### 1.1 First attempt — partial dataset (Ctrl+C'd early)

`sender_socket_sim.py` was manually interrupted mid-send, intentionally, as a quick sanity check before committing to the full run ("proses debugging," user's words).

| Metric | Count |
|---|---|
| Dataset total | 61 |
| Received (`sessions/uav_1/images/`) | 48 |
| Accepted (passed filters) | 45 |
| Stitched | 45 |

**Observation at the time**: result looked "faster and cleaner but slightly blurry" compared to the old diagonally-skewed orthomosaic.

**Why this observation was not usable on its own**: this run used a different (smaller, truncated) subset of frames than whatever produced the old skewed mosaic. Any visual difference here is confounded — could be the attitude fix, could just as easily be "fewer frames covering less ground, with the ROI-prediction small-motion assumption stressed further whenever a straggler frame's neighbor got dropped mid-sequence." This attempt was **discarded as inconclusive**, per the same isolate-variables principle used earlier in this project (see `docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md` / `docs/DRIFT_MISREGISTRATION.md`'s explicit separation of the two bugs).

### 1.2 Second attempt — full dataset (the real Task 9 test)

`sender_socket_sim.py` allowed to run to completion.

| Metric | Count | Source |
|---|---|---|
| Dataset total | 61 | `dataset/colleague_farmland_flight/*.jpg` |
| Received | 61 | `sessions/uav_1/images/` — **all 61 arrived, zero loss** |
| Accepted (passed both filters) | 51 | `GET /session/uav_1/status` → `accepted_count` |
| Rejected by filters | 10 | `61 - 51`; attitude (`>30°`) and/or GPS-distance (`<3m`) rejections, both working as designed |
| Stitched in this run | 50 | `last_stitch_count` at time of check |
| Not yet in a stitch batch | 1 | `images_since_last_stitch: 1` — accepted after the last trigger fired, waiting for the next threshold crossing |
| `intermediateResult_*.png` files | 49 | `combine()` runs `N-1` times for `N` images → 50 images → 49 steps |
| `matches_*.jpg` files | 49 | Same reasoning, one per `combine()` step |

All five numbers above are **mutually consistent** — every image is accounted for.

---

## 2. Investigated: "Watchdog isn't reading all the images" (user's initial suspicion)

**User's hypothesis**: only 49 images appear to have been processed (matches/intermediate file count), out of 61 sent — suspected the watchdog observer was dropping files, and suspected the auto-stitch queueing needs optimizing.

**Investigation** (evidence gathered before concluding, per this project's established debugging practice):

1. `ls sessions/uav_1/images/ | wc -l` → **61**. Watchdog/receiver captured every single file. No loss at ingestion.
2. `curl .../status` → `accepted_count: 51`. The gap from 61→51 is fully explained by the admission filters (`AttitudeThresholdFilter`, `GPSDistanceFilter`) rejecting 10 frames — this is correct, intended behavior, not a bug. (One rejection was directly observed in the terminal log: `[FILTER] Rejected (attitude 30.1° over threshold)` for the very last frame sent.)
3. `last_stitch_count: 50`, `images_since_last_stitch: 1` → of the 51 accepted, 50 were included in the one stitch run that actually executed; the 51st arrived after that stitch had already started reading its image list and is waiting for the next `auto_stitch_threshold` (5) crossing.
4. `combine()` in `Combiner.create_mosaic()` iterates `range(1, len(image_list))` — for 50 images that's exactly 49 iterations, hence 49 `matches_*.jpg` / `intermediateResult_*.png` files.

**Conclusion**: **not a watchdog bug.** Every received file is accounted for by either (a) correct filter rejection or (b) correct batching timing. The hypothesis was reasonable to raise, but the evidence doesn't support it — recorded here so this exact question doesn't need re-investigating later.

---

## 3. Real (separate) finding: auto-stitch batching doesn't run in clean threshold-sized chunks

While investigating §2, a genuine — but different — behavior was noticed: the stitch that ran processed **50** images in one shot, not a clean multiple of `auto_stitch_threshold` (5).

**Why this happens**: `StitchingSession.claim_stitch()` (Task 6) is an atomic single-flight lock — while one stitch is in progress, subsequent threshold-crossings don't launch a second stitch; they just keep appending to `accepted_images`. By the time the *next* `run_stitching()` call actually reads `session.accepted_images`, however many images have piled up since get swept into one batch. This is a **correct consequence of the atomicity fix** (prevents double-stitching, which was the original bug it was built to fix) — but it means batch sizes are unpredictable/lumpy rather than clean 5-image increments, especially when frames arrive faster than a stitch cycle completes.

**Status**: not a bug, but a legitimate design/optimization topic. **User has ideas for improving the queueing behavior — deferred, to be picked up later ("ntaran dulu").** Recorded as a to-do below.

---

## 4. Mosaic result: systemic skew is gone, but a sharp local kink remains

Comparing three images:
- `docs/WhatsApp Image 2026-09-07 at 13.32.51.jpeg` — old orthomosaic, produced by the colleague's buggy pipeline (degrees/radians double-conversion bug, `docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md`).
- `sessions/uav_1/output/finalResult.png` — new orthomosaic, produced by the fixed `src/flight_metadata.py` + `service.py` pipeline (this session's 61→51→50 run).

**Systemic diagonal skew — gone.** The old mosaic was rotated ~25–30° as a single rigid block, end to end — the signature of the attitude double-conversion bug (unrotation never actually applying real per-frame tilt correction). In the new mosaic, individual regions (the house/road/intersection area, the mid-flight field grid) are each internally **rectilinear / axis-consistent** — no whole-mosaic rotational bias. This is the expected, confirmed effect of the attitude-extraction fix.

**A sharp kink is visible instead.** Rather than smooth systemic rotation, the new mosaic shows the road/field pattern making a hard bend partway through — two locally-coherent regions meeting at an angle, rather than one smoothly rotated block. This is **not a new bug introduced by this fix** — it matches the failure mode already hypothesized in `docs/DRIFT_MISREGISTRATION.md` (single/few-frame misregistration from `src/Combiner.py`'s matching logic, a separate, already-tracked, not-yet-fixed issue). This was the **expected partial outcome** recorded in `docs/COMPOSABLE_ORCHESTRATOR_DESIGN.md` §5 and reconfirmed verbally earlier in this project: *"whole-mosaic rotational skew should reduce/disappear; the separate single-frame ghosting/mismatch artifact is expected to still be present."* That is exactly what happened.

---

## 5. Located a concrete repro case for `DRIFT_MISREGISTRATION.md`

Inspected `sessions/uav_1/output/matches/matches_43_44.jpg` through `matches_48_49.jpg` visually.

**`matches_43_44.jpg` — leading candidate for the kink's origin:**
- Only **4 matches total** — sitting exactly at `Combiner.py`'s acceptance floor (`if len(matches) < 4: skip`, line ~258). Any fewer and the frame would've been skipped instead of misregistered.
- One of the 4 match lines is a steep, near-diagonal line crossing almost the entire frame (top-left of the reference image to bottom-right of the incoming image) — geometrically implausible for a straight-line corridor flight, and visually the same pattern as the earlier-documented "red line" aliasing anomaly.
- Terrain at this point in the flight is still repetitive farmland (crop rows) — matches the SIFT-descriptor-aliasing-on-periodic-texture hypothesis in `DRIFT_MISREGISTRATION.md` §3 exactly: too few genuine matches, one of them geometrically wrong, `estimateAffinePartial2D` forced to fit a transform to sparse/corrupted data.

**`matches_45_46.jpg` onward — match count jumps sharply upward.** The flight path starts crossing a village/road area (rooftops, trees visible in the corner of the frames) — non-repetitive texture gives SIFT many more distinctive, reliable features to match against. Matching quality visibly improves from this point on.

**Conclusion**: frame pair **43↔44** is the strongest candidate for where the kink originates, and is now a **real, concrete repro case** — not just a hypothesis — for validating the `DRIFT_MISREGISTRATION.md` §9 checklist fixes (GPS sanity check + inlier-ratio check) once that work starts.

`DRIFT_MISREGISTRATION.md` §8's open questions (1: check match visualization for the bad pair: done, confirms low/sparse pattern; 2: check whether match count was at-or-near the floor: confirmed, exactly 4) have been updated to reflect this.

---

## 6. To-Do List (recorded, not yet started)

- [ ] **Continue `docs/DRIFT_MISREGISTRATION.md` §9 checklist next** — the GPS translation sanity check + RANSAC inlier-ratio check in `src/Combiner.py`. Frame pair `43↔44` from this session's `sessions/uav_1/output/matches/` and `intermediateResult_*.png` is now available as a concrete before/after validation case (re-run stitching with the fix applied against the same dataset, confirm the kink at that pair specifically is resolved).
- [ ] **Auto-stitch queueing/batching optimization** — covers two related "N-1" problems observed this session, both pointing at the same underlying need for a proper queue instead of the current read-whatever's-accumulated-so-far approach:
  - **(a) Orphaned trailing image.** `run_stitching()` currently processes whatever has accumulated in `accepted_images` by the time it starts reading, rather than clean `auto_stitch_threshold`-sized batches (see §3 above). A direct consequence: an accepted image that arrives just after a stitch batch closes (this run's 51st image, `images_since_last_stitch: 1`) sits unstitched indefinitely unless enough further images arrive to cross the next threshold, or someone manually triggers `/stitch`. A proper queue should guarantee every accepted image eventually gets included in some stitch, not just "whichever ones happened to be in the list at read-time."
  - **(b) `combine()`'s N-1 processing characteristic.** Of N images handed to `Combiner`, only N-1 actually go through the full per-frame pipeline (`combine(index)` for `index in range(1, N)`) — `image_list[0]` is used directly as the seed for `result_image` and is never independently feature-matched or warped against anything. This is an inherent property of the current sequential-chaining algorithm design (`docs/TECHNICAL.md` §4.4), not a queueing bug by itself, but it interacts with (a): if a queue design changes how batches are split/re-anchored, each new batch's first image inherits this same "never independently validated" characteristic — worth designing (a) and (b) together rather than treating them as fully separate problems.
  - User has ideas for this, to be scoped and designed in a future session.
- [ ] **Temporary GCS integration of this stitching modular service** — user wants to integrate this repo's updated stitching service into the base station (`gcs_docs.md`'s `app/routers/stitching.py` + `stitching_service/`) now that the attitude-correction pipeline is in place, ahead of finishing the `DRIFT_MISREGISTRATION.md` work. Four things flagged to pay attention to on the GCS side:
  1. **Pipeline execution order/state needs re-syncing.** The stitching router/modular service on the GCS side was written against the OLD pipeline shape (no flight-metadata correction step, `dataMatrix`'s Yaw/Pitch/Roll always zero — see `docs/gcs_docs.md` §4.3.4, `create_mosaic()` only fills columns 0-2). Since this repo's pipeline now has a real attitude-extraction + filtering stage (`src/flight_metadata.py`) sitting in front of `Combiner`, the GCS-side execution order and session state need to be checked against this new shape before wiring it in — not a drop-in swap.
  2. **Auto-stitch queueing optimization becomes more urgent once this is real GCS integration**, not just local testing (ties to the queueing to-do above). **Also flagged, not yet RnD'd at all: parallelism for multi-UAV stitching** — running multiple UAVs' stitch pipelines concurrently hasn't been investigated yet.
  3. **UDP vs TCP for the image receiver socket needs a re-comparison.** The colleague's PoC (`PROGRAM_JUANG/PROGRAM-SENDER1/`) sends images over UDP (fragmented, `sender.py`/`receiver.py`); this repo's `receiver_socket.py` uses TCP. Worth noting while comparing: **GCS's own live *video* pipeline already uses UDP** (`docs/gcs_docs.md` §2 Fleet topology table + §3.4 — ports **5600** for UAV1 and **5601** for UAV2, JPEG frames via `VideoReceiver`) — this is a separate, already-working precedent for UDP image transport in this same codebase, worth reading closely (`docs/gcs_docs.md` §3.4) before deciding whether the stitching image channel should follow the same pattern or stay TCP. *(Note: the port number "5000" mentioned when this to-do was recorded wasn't found anywhere in the current `docs/gcs_docs.md` — the verified port numbers from that doc are 5600/5601. Re-confirm against the live GCS repo/config before relying on any specific port number here.)*
  4. **Roadmap after UDP is validated**: video stream/HUD compression via WebRTC — matches `docs/gcs_docs.md` §3.4's own explicit note that the video pipeline is "Currently MJPEG-over-WebSocket... WebRTC migration is an unimplemented roadmap item." Natural next step once the image-transport protocol question (item 3) is settled.

---

*Related reading: [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md), [BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md](./BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md), [COMPOSABLE_ORCHESTRATOR_DESIGN.md](./COMPOSABLE_ORCHESTRATOR_DESIGN.md), [superpowers/plans/2026-09-15-composable-orchestrator.md](./superpowers/plans/2026-09-15-composable-orchestrator.md).*
