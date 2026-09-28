# BIMA GCS — Contribution Roadmap

Working document surveying open roadmap items from `README.md` and unimplemented stubs found in the current codebase (audited 2026-09-14, against commit `ad7f843`). Intended as a menu for picking up a second/third area of ownership beyond an existing feature assignment.

## Ownership already claimed (out of scope for this doc)

- **VISION-LIVESTITCH image stitching + its frontend panel** (`app/routers/stitching.py`, `stitching_service/`, `gcs_js/src/components/stitching/*`) — owned by the reader of this document.
- **Swarm coordination / AI Decision Layer** (`app/services/mavlink/apf_coordinator.py`, `swarm_coordinator.py`, `app/routers/swarm.py`, `gcs_js/src/components/control/SwarmControlPanel.tsx`) — introduced in commit `ad7f843`, currently under active development by another team member. Not listed as a candidate below, and any work below must not modify these files.

---

## Owned Work — Image Stitching Algorithm Refactor & Optimization

**Not a pickup candidate — this is the reader's own committed scope, logged here for tracking alongside everything else.** Findings below come from a direct read of `stitching_service/src/{Combiner.py,blending.py,redundant_filter.py,geometry.py,utilities.py}`, not from the paper citations in the docstrings alone.

### Current algorithm shape

`Combiner.create_mosaic()` (`Combiner.py:325-337`) runs a **sequential pairwise chain**: image 0 is the seed, then each subsequent image is SIFT-matched against the *previous running mosaic*, homography-estimated, warped onto a growing canvas, and feather-blended in, one at a time, in capture order. Per step (`combine()`, `Combiner.py:200-323`):

1. Predict an ROI crop on the incoming frame from the last relative homography (`__predict_roi`, forward-motion heuristic)
2. `SIFT_create(1800)` feature detection on both the previous frame and the new frame's ROI
3. `cv2.BFMatcher().knnMatch(k=2)` + manual ratio test (0.55)
4. `cv2.estimateAffinePartial2D` → fallback to `cv2.findHomography(..., RANSAC)`
5. Chain into a running global homography (`H_global_prev`)
6. Full-canvas `cv2.warpPerspective` of both the entire running mosaic *and* the new frame
7. `ROIfeatherBlender._roi_feather_blend` composite
8. Unconditional disk write of an intermediate PNG + a match-visualization JPEG

### Concrete bottlenecks / refactor targets found

1. **Redundancy/keyframe filtering exists but is never invoked.** `redundant_filter.py` implements two full, paper-cited algorithms — `redundancy_removal_he2024` (BRISQUE-quality-based redundant-frame pruning) and `keyframe_selection_yuan2024` — and `Combiner.py:9` imports both, but neither is called anywhere in `combine()` or `create_mosaic()`. This is the single highest-leverage fix: total pipeline cost is dominated by the number of frames processed (each one full SIFT + BFMatcher + full-canvas warp), so filtering near-duplicate frames *before* the sequential chain starts directly cuts N rather than optimizing per-step constant factors. Wiring this in should be the first change, before touching anything else below.
2. **Features get computed twice per frame.** `image1`'s keypoints/descriptors are recomputed from scratch every iteration (`__detect_features(image1)`, `Combiner.py:221`) even though that exact frame was `image2` — and already had `__detect_features` run on it — in the previous iteration. Caching `(kp, descriptors)` keyed by frame index and reusing it as `image1`'s features next step removes roughly half the feature-detection wall time for free, no algorithmic change needed.
3. **Full-mosaic re-warp every step, canvas grows every step.** `_warp_images` (`Combiner.py:159-177`) calls `cv2.warpPerspective` on `self.result_image` — the *entire accumulated mosaic* — on every single `combine()` call, and the canvas bounds grow with each new frame (`__compute_canvas_bounds`). Warp cost scales with pixel count, so total warp cost across a full sequence grows faster than linearly in frame count — this is the classic "naive growing panorama" trap. Options: warp only the new frame into a fixed/pre-allocated canvas sized from flight-path bounds instead of recomputing bounds incrementally, or move to a tile-based compositing structure that doesn't require touching the whole existing mosaic per step.
4. **No global optimization — drift is self-acknowledged in the source.** `combine()` has an explicit comment: *"Assume the order in which the images are given is the best order. This introduces drift!"* (`Combiner.py:203-204`). Pure sequential pairwise chaining has no loop closure or global bundle adjustment, so homography error accumulates monotonically over a long survey run — worse the longer the mission. A bounded refactor (not full bundle adjustment) would be periodic re-anchoring against GPS pose data already available in `dataMatrix` (used today only for the initial unrotation in `geometry.computeUnRotMatrix`) to correct drift periodically rather than trusting pure visual chaining indefinitely.
5. **Richer blending strategies are implemented but dead code.** `blending.py` has `PyramidBlender` (multi-band Laplacian blending), `hybridBlender`, `AdaptiveWeightedFusion`, and `incrementalFusion` fully coded (`blending.py:7-349`), but `Combiner.py:299` only ever calls `ROIfeatherBlender._roi_feather_blend` — a simpler feather blend. Feather blending alone handles exposure/parallax mismatches worse than multi-band blending at overlap seams. Worth a visual A/B on real survey output to see if switching to `hybridBlender.hybrid_blend(fast_mode=True)` (already has a documented perf/quality toggle) measurably improves seam quality for an acceptable cost.
6. **Naive strided downsampling aliases the source image.** `img[::5, ::5]` (`Combiner.py:54`) is nearest-neighbor decimation, not area-averaged — `cv2.resize(img, ..., interpolation=cv2.INTER_AREA)` is the correct operation for downsampling and costs about the same; likely improves SIFT keypoint repeatability at the same 5x reduction for free.
7. **Fully single-threaded, no parallelism anywhere.** `create_mosaic()`'s loop (`Combiner.py:330-334`) is strictly sequential. Feature detection *is* embarrassingly parallel across independent frames (OpenCV's C++ SIFT/BFMatcher calls release the GIL), so a `ThreadPoolExecutor` pre-pass that detects features for every frame up front — which also directly solves bottleneck #2 above — would let real wall-clock detection work happen concurrently before the necessarily-sequential matching/warping/blending chain begins.
8. **Unconditional per-step disk I/O inside the hot loop.** Every `combine()` call writes an intermediate PNG (`Combiner.py:305-307`) and a full match-visualization JPEG (`Combiner.py:317-323`) regardless of whether anyone's watching. Fine for debugging a single run, wasteful for a long automated survey session. Should be gated behind a debug flag — `app/routers/stitching.py`'s `StitchConfig` already has toggles like `folder_monitoring_enabled`, a natural place to add `debug_output_enabled`.
9. **ROI-prediction heuristic may not hold for the swarm's actual flight paths.** `__predict_roi` (`Combiner.py:128-157`) explicitly documents its assumption: *"camera motion is mostly forward and the scene is mostly planar."* The new `app/services/mission/survey_generator.py` (commit `ad7f843`, colleague's swarm work) generates boustrophedon/lawnmower survey paths with 180° turns between sweep lines. At each turn, the forward-motion assumption breaks, the predicted ROI crop can miss real overlap, and `combine()`'s `< 4 matches` fallback (`Combiner.py:258-260`) silently skips that frame pair — meaning turn-point frames could get silently dropped from the mosaic on real survey missions. Worth flagging to whoever owns survey-path generation, and worth a runtime check (e.g. detect the fallback-to-full-image low-match case and retry once before giving up) rather than a silent skip.
10. **`preprocessing.py` is an empty file (0 lines).** Either a stale placeholder to delete, or the intended home for the preprocessing logic currently inline in `Combiner.__preprocess_images` (`Combiner.py:48-70`) — worth deciding as part of the refactor rather than leaving a dead empty module in the tree.

### Suggested sequencing

Wire in #1 (redundancy filtering) first — it reduces N before any per-step optimization matters. Then #2 (feature caching) and #6 (proper downsampling) as cheap, safe, behavior-preserving wins. Then evaluate #5 (blending quality) and #9 (ROI heuristic robustness) together since both affect visual correctness of the output, not just speed. #3 (canvas growth) and #4 (drift correction) are the biggest architectural changes — do them last, once the cheaper wins are banked and there's a stable baseline to measure against.

### Tradeoffs

- Feature caching, proper downsampling, and gating debug I/O are pure wins — no quality/robustness tradeoff, just implementation time.
- Wiring in redundancy filtering changes output determinism (fewer frames stitched than today) — needs a before/after visual comparison on a real dataset to confirm quality doesn't regress, not just a speed measurement.
- Canvas/drift refactors are the highest-value, highest-risk items — they touch the core accumulation logic (`H_global_prev`) that everything else depends on, so should land behind their own test pass on a known survey dataset with a way to visually diff before/after mosaics, not just unit-tested in isolation.
- Parallelizing feature detection adds concurrency-related complexity (thread safety of shared OpenCV objects, ordering guarantees) for a pipeline that's otherwise been straightforwardly sequential — worth confirming the speedup is actually worth it on realistic frame counts before committing, since I/O-bound disk writes (#8) might be a bigger practical win for less risk.

### Effort estimate

Items 1, 2, 6, 8, 10: half a day combined, low risk, good first PR. Item 5 (blending A/B): half a day plus eyeballing real output. Items 3, 4, 9: multi-day each, architecture-level, should be scoped as separate PRs with their own before/after validation rather than bundled together.

---

## Candidate 1 — Flight Command Completion (RTL / Takeoff / Guided Goto)

**Status: ready now, no design decisions blocking it.**

### Current state

The MAVLink command bridge implements `arm()`, `disarm()`, and `set_mode()` end-to-end (request → ACK future → timeout), but four methods are explicit stubs:

```python
# app/services/mavlink/command_bridge.py
async def return_to_launch(self, slot: int) -> bool:                      # line 180
    raise NotImplementedError("TODO: implement in control feature task")

async def takeoff(self, slot: int, altitude_m: float) -> bool:            # line 184
    raise NotImplementedError("TODO: implement in control feature task")

async def goto(self, slot, latitude_deg, longitude_deg, altitude_m):      # line 188
    raise NotImplementedError("TODO: implement in control feature task")

async def handle_command_ack(self, slot: int, message: Any) -> None:      # line 439
    raise NotImplementedError("TODO: implement in control feature task")
```

The REST layer already routes to them and will raise the same error:
- `POST /api/control/{slot}/rtl` — `app/routers/control.py:305-310`
- `POST /api/control/{slot}/takeoff` — `app/routers/control.py:313-319`
- `POST /api/control/{slot}/goto` — `app/routers/control.py:322-328`

The frontend mirrors the stub 1:1 so both sides fail the same way today:
- `gcs_js/src/hooks/useUavControl.ts:52-59` — `rtl()`, `takeoff()`, `goto()` all `throw new Error(TODO_MESSAGE)`
- `gcs_js/src/lib/controlApi.ts:128-131` — same sentinel
- UI already built and visually wired, just calling into the throw: `RtlButton.tsx`, `GuidedGotoControl.tsx`, `ArmDisarmButton.tsx`, `ModeSelector.tsx`, `ControlPanel.tsx` (each has a `// TODO: implement in control feature task` marker at/near line 6-13)

### Technical approach

All three follow the exact transaction shape already proven by `set_mode()` (`command_bridge.py:132-178`):

```python
async with self._router.transaction_lock(slot):
    ack_future = self._router.expect(slot, {"COMMAND_ACK"}, predicate)
    master.mav.command_long_send(master.target_system, master.target_component, cmd_id, 0, p1..p7)
    ack = await asyncio.wait_for(ack_future, timeout=5.0)
```

Per-command specifics:

| Command | MAVLink primitive | Notes |
|---|---|---|
| `return_to_launch` | `MAV_CMD_NAV_RETURN_TO_LAUNCH` via `command_long_send`, **or** reuse `self.set_mode(slot, "RTL")` | The mode-based approach is a ~3-line wrapper around existing, tested code — cheapest correct implementation. The dedicated command is more "textbook" but duplicates ACK-wait boilerplate for no functional gain on ArduPilot, since RTL is a first-class flight mode there. |
| `takeoff` | `MAV_CMD_NAV_TAKEOFF`, param7 = altitude_m | Copter-only in practice (ArduPilot rejects it on plane in most modes). Should validate the target UAV is a copter (slot 3/4, see `gcs_js/src/config/agents.ts` `COPTER_IDS`) before sending, otherwise the ACK will just come back rejected — worth a clearer client-side error than a raw MAVLink rejection string. |
| `goto` | `SET_POSITION_TARGET_GLOBAL_INT` (streamed, no ACK) or `MAV_CMD_DO_REPOSITION` (command_long, ACK'd) | `DO_REPOSITION` fits the existing ACK-future pattern cleanly and only requires GUIDED mode; `SET_POSITION_TARGET_GLOBAL_INT` is the more common real-world choice for continuous guided nav but doesn't produce a `COMMAND_ACK`, meaning the response contract (`CommandAckResponse`) would need to either fake success on send-without-error, or you'd need to correlate against the next `MISSION_CURRENT`/position telemetry instead — a genuinely different completion signal than every other command in this file. Recommend `DO_REPOSITION` first to keep the codebase's completion semantics uniform; revisit `SET_POSITION_TARGET_GLOBAL_INT` only if `DO_REPOSITION` proves too coarse for real guided-flight needs. |
| `handle_command_ack` | generic dispatcher | Currently unreachable in practice — `arm`/`disarm`/`set_mode` each register their own inline ACK predicate via `router.expect()`, so this method isn't actually called by the working paths. Once `rtl`/`takeoff`/`goto` exist, decide whether they follow the same "each call registers its own future" pattern (consistent, recommended) or centralize through this method (requires a slot-keyed command-in-flight registry). Recommend keeping the existing per-call pattern and either deleting this stub or repurposing it as a catch-all logger for *unexpected* ACKs (i.e. ones nobody was waiting on).

### Tradeoffs

- **Pro:** small, bounded, full-stack (backend + frontend both touched), reuses a pattern that's already been through review once (`set_mode`), doesn't touch swarm code, unblocks nothing else being worked on so there's no merge-conflict risk with the swarm branch.
- **Con:** genuinely safety-relevant — `takeoff` and `goto` actuate real aircraft. Needs SITL testing (`companion_bridge.py --fc-connection udp:127.0.0.1:14551`, per `README_mission.md`) before any field test, and force-arm/force-disarm precedent in this file (`force=True` sends param2=21196) shows the team is comfortable with escape hatches — the same judgment call should be made explicit for takeoff altitude bounds and goto radius limits (e.g. reject a `goto` more than N meters from current position without a second confirmation), since nothing in `schemas/control.py` currently caps those values.
- **Note on interaction with swarm work:** the swarm coordinator's abort path currently only calls `set_mode("LOITER")`, never RTL (confirmed via prior audit). Once `return_to_launch` exists, whether swarm abort should switch to using it is **the swarm owner's decision, not this task's** — implement the primitive, don't wire it into `swarm_coordinator.py` unprompted.

### Effort estimate

Half a day to a day for a working implementation + SITL pass, given the pattern is copy-adapt rather than novel design. Frontend re-enabling is mechanical once the backend contract is solid (remove the `throw`, wire to `controlApi.ts`, the buttons already exist).

---

## Candidate 2 — WebRTC Video/HUD Transport

**Status: greenfield. Zero code exists today (`grep -ri webrtc` across the repo returns only the README roadmap line and this document). This is architecture-level work, not a stub pickup — raise it with the team before starting.**

### Current state

Video pipeline today is MJPEG-over-WebSocket, not WebRTC:

```
UAV → UDP JPEG packets → VideoReceiver (app/services/video/receiver.py)
    → decodes to np.ndarray, stores as VideoReceiver.latest_frame
    → MultiStreamManager (app/services/video/manager.py) re-encodes to JPEG
    → raw bytes broadcast over /ws/video/{port} WebSocket (app/routers/video.py)
    → frontend VideoPanel.tsx reads binary blob, draws to <canvas> per frame
```

No codec, no adaptive bitrate — each frame is a full independent JPEG regardless of link quality. This is the direct motivation for the roadmap line ("Kompresi stream Video/HUD via WebRTC untuk skenario koneksi minim" — compression for low-bandwidth links), since this project explicitly targets Tailscale-based remote field deployments (`TAILSCALE_ENABLED` in `.env`, `get_tailscale_ip()` in `app/main.py`).

### Confirmed cross-feature dependency

The stitching feature (owned by the reader) reads frames from **`VideoReceiver.latest_frame` directly** (`app/routers/stitching.py:541-551`, `wait_for_stream_frame()` → `video_manager_instance.get_latest_frame(port)` → `receiver.py:158`), **upstream of** the WebSocket JPEG broadcast. A WebRTC migration that only replaces the broadcast layer (`MultiStreamManager` → `aiortc` track feed) leaves this dependency intact. A migration that also restructures `VideoReceiver`'s frame-storage internals (e.g. swapping the plain `latest_frame` attribute for an async queue to feed an `aiortc.VideoStreamTrack`) would need to preserve `get_latest_frame(port)`'s synchronous-read contract, or `capture-stream` breaks silently — there's no test coverage linking these two subsystems today.

### Technical approach

- **Backend:** `aiortc` (Python WebRTC) is the natural fit — pure-Python, asyncio-native, integrates with FastAPI's existing event loop without a separate media server process. Feed `VideoReceiver`'s decoded frames into a custom `VideoStreamTrack`. Needs an SDP offer/answer signaling exchange — can reuse the existing `/ws/video/{port}` WebSocket as the signaling channel instead of adding a new one, keeping the connection-per-port model intact.
- **Frontend:** replace the canvas-blit read loop in `VideoPanel.tsx` with a real `<video>` element + `RTCPeerConnection`. The detection-overlay logic in `HudCanvas.tsx` is already a separate canvas layer drawn on top of the video surface — it survives unchanged regardless of how the underlying video element gets its pixels.
- **Alternative considered:** a dedicated SFU (mediasoup, Janus) instead of `aiortc` in-process. Rejected as a starting point — adds a second service to deploy/monitor for a 4-camera-max fleet, when `aiortc` handles peer-to-peer fan-out for this scale without extra infrastructure. Revisit only if client count per stream grows well past what one `aiortc` process comfortably serves.

### Tradeoffs

- **Pro:** directly serves the project's stated field-deployment use case (weak/remote links over Tailscale); real codec compression vs. raw JPEG-per-frame is a substantial bandwidth win.
- **Con:** touches shared infrastructure every video-consuming feature depends on (main dashboard panels, stitching capture, any future detection overlay). Not something to build quietly in a branch — needs a short design note and a heads-up to whoever else touches `services/video/` before starting, specifically to preserve the `latest_frame` contract stitching relies on.
- **Con:** `aiortc` pulls in its own native dependency chain (`aioice`, `pylibsrtp`, `av`/PyAV which wraps ffmpeg) — heavier install footprint than the current pure-OpenCV pipeline, worth checking against whatever the Raspberry Pi companion / field laptop deployment story is before committing.
- **Con:** no existing test or CI coverage for the video pipeline at all today, so this would be introducing a hard-to-verify subsystem without an existing safety net — worth pairing with at least a manual verification checklist (documented in the same PR) given there's no automated way to assert "the frame the operator sees is not stale."

### Effort estimate

Multi-day to multi-week depending on how much adaptive-bitrate/reconnection robustness is wanted beyond a functional happy path. Recommend scoping an MVP (`aiortc` signaling over existing WS, one working panel end-to-end, `HudCanvas` overlay confirmed still working) as a first PR rather than all four panels + reconnection handling in one pass.

---

## Candidate 3 — Centralized Operator Authentication

**Status: greenfield. Zero auth code anywhere in the repo.**

### Current state

```python
# app/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],           # Restrict in production to known origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Every REST and WebSocket endpoint is unauthenticated — including `arm`/`disarm`/`mode`/mission-upload on `/api/control/{slot}/*`, and now the swarm `start`/`abort` endpoints. Anyone who can reach the bound host/port (or the Tailscale network) can command real aircraft. `allow_origins=["*"]` combined with `allow_credentials=True` is also flagged directly in the source as needing tightening "in production."

### Technical approach

- **Backend:** given FastAPI, the lowest-friction path is a dependency-injected auth check (`fastapi.Security` / `Depends`) on the `api_router`/`router` instances — likely a shared bearer-token or session-cookie dependency applied at router-include time in `app/main.py`, rather than decorating every individual endpoint. For WebSocket routes (`/ws/control/{slot}`, `/ws/telemetry`, `/ws/video/{port}`, `/ws/swarm`, `/ws/stitching/{session_id}`), auth has to happen at `websocket.accept()` time — token as a query param or a subprotocol header, since WS doesn't carry normal Authorization headers on the initial browser-driven handshake.
- **Given single-operator-team-in-the-field reality today** (not a multi-tenant SaaS), a pragmatic v1 is a single shared operator token issued via `.env` (`OPERATOR_TOKEN` alongside existing `MAVLINK_HOSTS`-style config), checked on every REST call and WS handshake — not full user accounts/roles. That covers the actual threat model (stray device on the Tailscale network sending arm commands) without building a user-management system nobody asked for.
- **Frontend:** token needs to live somewhere across page loads — `useGCSStore.tsx` already has a `localStorage`-backed pattern (`bima_gcs_configured`, per-UAV connection configs) to extend, plus every `fetch`/WebSocket call site in `lib/controlApi.ts`, `lib/swarmApi.ts`, `lib/stitchApi.ts`, and each `hooks/use*WebSocket.ts` needs the token attached.

### Tradeoffs

- **Pro:** closes a real, currently-live gap — arm/disarm and swarm mission start are exploitable by anything on the network today with zero auth.
- **Con:** touches literally every API call site in the frontend (four `lib/*Api.ts` files, every WebSocket hook) — high surface area even though each individual change is small, meaning it's easy to miss one endpoint and leave a hole.
- **Con:** "centralized operator auth" in the README could mean anything from a shared token (hours of work) to full per-operator accounts with audit logging (much bigger, arguably a separate roadmap item on its own) — needs a scope conversation with the team before starting, this doc can't resolve that ambiguity alone.
- **Con:** WebSocket auth-at-handshake is the fiddliest part technically and easy to get subtly wrong (e.g. accepting the connection before validating, leaking a timing side-channel) — worth a design review before merging even the "simple" version.

### Effort estimate

A shared-token v1 (recommended scope): 1-2 days backend + 1 day frontend wiring across the four API modules. Full RBAC/user-account version: significantly larger, treat as a separate future roadmap item, not bundled into this one.

---

## Resolved — Design System

`SOFT_DESIGN_SYSTEM.md` is confirmed live in the shipped code, verified directly against source rather than assumed:

- `gcs_js/src/app/globals.css:20` `--bg-base: #0E0E0E` (dark) / `:120` `#FAFAF8` (light) — matches the soft doc's neutral token table exactly.
- `globals.css:78-82` — radius scale `6/10/14/18px` — the soft doc's scale, not `DESIGN_SYSTEM.md`'s square 2px cockpit corners.
- `gcs_js/src/app/layout.tsx:2,8,13` — `Inter` + `JetBrains_Mono` loaded via `next/font/google` — the soft doc's font stack, not `DESIGN_SYSTEM.md`'s Barlow Condensed / IBM Plex Mono.

`DESIGN_SYSTEM.md` — despite containing a detailed "post-render critique" section describing real browser testing — describes a proposal that was not the one ultimately shipped. It should be treated as superseded/historical, not consulted for new component work. Worth a quick note to the team to delete or explicitly mark it deprecated so future contributors don't get misled the way this document initially was — but that's a one-line cleanup, not a contribution-sized task, and it's off the candidate list below.

For your stitching UI panels specifically: style against `SOFT_DESIGN_SYSTEM.md`'s tokens (rounded 6-18px radius, `--bg-base`/`--bg-elevated`/etc., Inter + JetBrains Mono) and cross-check against `globals.css`'s actual `:root` block for the current token names, since some naming may have evolved slightly since the doc was written.

---

## Suggested order

1. **Flight Command Completion** — start here. Bounded, well-understood, no dependency on anyone else's in-flight work, immediately useful (also removes dead buttons from the UI).
2. **WebRTC** and **Auth** — both real, both valuable, both need a short team design conversation before code starts given their shared-infrastructure blast radius. Raise both in the same conversation since they touch overlapping call sites (every API/WS client) and could reasonably be sequenced together (auth-token-on-every-call and WebRTC-signaling-on-the-same-WS are not unrelated).
