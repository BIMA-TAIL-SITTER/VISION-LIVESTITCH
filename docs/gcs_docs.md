# BIMA SWARM UGM — Ground Control Station: Technical Overview

Comprehensive technical reference for the full ground station system, with a deep-dive emphasis on the image stitching subsystem. Written 2026-09-15 against commit `ad7f843` on `main`. This document describes **how the system currently works**, top to bottom; `CONTRIBUTION_ROADMAP.md` describes **what to build or change next**, including a full actionable refactor plan for the stitching engine. Read both together.

---

## 1. Executive Summary

BIMA GCS is a ground control station for a **4-UAV heterogeneous swarm**: two fixed-wing scouts carrying live video + onboard edge-AI detection, and two quadcopters flying autonomous survey missions coordinated through a dual-copter collision-avoidance layer. The system is split into an independently-deployable FastAPI backend (all network I/O, MAVLink, video ingest, mission logic) and a Next.js frontend (pure client, renders everything the backend streams over WebSocket/REST). It's designed for field deployment over Tailscale-mediated remote networks, with offline satellite mapping so the operator doesn't depend on live internet access at the field site.

Five major subsystems compose the backend: the MAVLink command/telemetry stack, the UDP video pipeline, offline GIS tile serving, a dual-path mission-control system (direct MAVLink + Raspberry Pi companion-bridge relay), and the image stitching orthomosaic engine — the subsystem this document treats in the greatest depth.

---

## 2. System Architecture & Deployment Topology

```
┌────────────────────────────┐         ┌───────────────────────────────────┐
│  Next.js 15 / React 19       │  HTTP   │  FastAPI Backend                     │
│  Frontend — port 3000         │◄───────►│  app/  — port 8000                     │
│  gcs_js/                      │  WS     │                                       │
└────────────────────────────┘         └──────────────┬────────────────────┘
                                                          │
             ┌───────────────────────┬─────────────────────┼─────────────────────┬──────────────────────┐
             │                        │                      │                      │                       │
      ┌──────▼──────┐        ┌────────▼────────┐   ┌─────────▼─────────┐   ┌────────▼────────┐    ┌─────────▼─────────┐
      │ MAVLink TCP  │        │ UDP Video (JPEG) │   │ UDP Edge-AI JSON   │   │ Raspi Companion  │    │ Offline MBTiles    │
      │ 4 UAV slots  │        │ Fixed-wing 1/2    │   │ Detection stream    │   │ Bridge — copters  │    │ SQLite (Git LFS)   │
      │ (TCP :576x)  │        │ (ports 5600/5601) │   │ (paired json_port)  │   │ 3/4, mission UDP  │    │ + Esri auto-cache  │
      └──────────────┘        └──────────────────┘   └────────────────────┘   └──────────────────┘    └────────────────────┘
```

The backend owns every network connection — no browser ever talks MAVLink or UDP directly. All live data reaches the frontend exclusively through the backend's WebSocket channels (`/ws/video/{port}`, `/ws/telemetry`, `/ws/system`, `/ws/control/{slot}`, `/ws/stitching/{session_id}`, plus the newer `/ws/swarm`), each backed by a corresponding `/api/*` REST surface for one-shot actions and state queries.

**Deployment model:** the two halves run as independent processes (`uvicorn app.main:app` and `npm run dev`, or via `docker-compose`), communicating over plain HTTP/WS on the local network or a Tailscale mesh. `/api/config` lets the frontend self-discover the backend's Tailscale IP and port at runtime rather than hardcoding it, which is what makes the field-deployment story (laptop + Tailscale + remote UAVs) work without per-site reconfiguration.

### Fleet topology

Defined in `gcs_js/src/config/agents.ts`, with matching slot numbering baked into the backend (`UAV_SLOTS = (1, 2, 3, 4)` in `telemetry_bridge.py`):

| Slot | Type | Video | Detection | Control path | Default MAVLink port | Default video port |
|---|---|---|---|---|---|---|
| 1 | Fixed wing | Yes | Yes (edge AI, JSON UDP) | Direct MAVLink TCP | 5761 | 5600 |
| 2 | Fixed wing | Yes | Yes (edge AI, JSON UDP) | Direct MAVLink TCP | 5762 | 5601 |
| 3 | Copter | No | No | MAVLink TCP + companion-bridge mission UDP | 5763 | — |
| 4 | Copter | No | No | MAVLink TCP + companion-bridge mission UDP | 5764 | — |

This asymmetry shapes the whole system: fixed-wing UAVs are treated as live-sensing scouts (video, detection, direct manual control), copters are treated as autonomous mission platforms (survey paths, swarm coordination, indirect control via companion computer). The image stitching service's frame sources — both live capture and file upload — are agnostic to this split, but in practice the imagery being stitched originates from the fixed-wing video streams or from post-flight batch uploads, not from the copters (which have no video path in this system).

---

## 3. Backend Architecture (`app/`)

### 3.1 Application entry point & dependency wiring (`app/main.py`)

FastAPI app built around an `asynccontextmanager` lifespan function. At **module import time** (before the app even starts), singleton service instances are constructed: `WebSocketManager`, `MultiStreamManager` (video), `MavlinkTelemetryBridge`, `MavlinkMessageRouter`, `MavlinkCommandBridge`, `MavlinkParamBridge`. Message-type routing is wired at this point too — the telemetry bridge subscribes to every message type via a wildcard (`{"*"}`), while the param bridge subscribes only to `AUTOPILOT_VERSION`/`HEARTBEAT`/`PARAM_VALUE`.

**Dependency injection pattern:** this codebase does *not* use FastAPI's `Depends()` system for service access. Instead, each router module (`control.py`, `video.py`, `telemetry.py`, `stitching.py`, `system.py`) declares module-level globals initialized to `None` (e.g. `command_bridge_instance = None`), and `main.py`'s `lifespan()` function assigns the real singleton into each module's namespace after startup begins. Route handlers check `if xxx_instance is None: raise 503` before use. This is a deliberate, if unconventional, choice — worth knowing before assuming `Depends()`-style patterns apply anywhere in this codebase.

**Startup sequence** inside `lifespan()`: create `snapshots/` directory → inject service instances into router globals → call `stitching.startup()` (discovers existing stitching sessions from disk, captures the running event loop for cross-thread scheduling) → start the single MAVLink message-router read loop → spawn the telemetry broadcast task. **Shutdown**: cancel the telemetry task, stop all video receivers, shut down stitching (cancels in-flight stitch tasks, stops folder watchers), stop the param bridge, stop the message router.

CORS is wide open (`allow_origins=["*"]`, `allow_credentials=True`) with an inline comment flagging it for production hardening — see `CONTRIBUTION_ROADMAP.md` Candidate 3 for the full authentication gap analysis.

### 3.2 Central WebSocket hub (`app/services/websocket/manager.py`)

`WebSocketManager` is the single point every live-data broadcast passes through, regardless of channel. It tracks four independent client-set namespaces: `_video_clients` (keyed by UDP port, since multiple video streams run concurrently), `_telemetry_clients` (flat set, one global telemetry channel), `_system_clients` (flat set, log/event stream), `_control_clients` (keyed by UAV slot, since mission/param progress is per-vehicle).

Broadcast is fan-out via `asyncio.gather(..., return_exceptions=True)` — every connected client in a namespace gets the message concurrently, and any client whose `send()` raises (typically a closed/dead connection) is silently pruned from the set on the same pass, rather than being handled as an error. This means a single slow or dead WebSocket client cannot block delivery to the others, and disconnects are detected lazily on next-broadcast rather than requiring an explicit disconnect handshake.

### 3.3 MAVLink stack (`app/services/mavlink/`)

**Single-reader invariant.** Exactly one coroutine reads each UAV's raw MAVLink connection — `MavlinkMessageRouter`'s internal loop — and fans messages out to registered handlers by message type. This is a hard architectural constraint documented directly in `command_bridge.py`'s class docstring: *"this class never reads the native pymavlink connection"* — every other service, including the command bridge that sends commands, only ever reads via `router.expect(slot, types, predicate)`, which returns an `asyncio.Future` resolved the next time a matching message arrives. This avoids the classic bug of two coroutines racing on the same socket read.

**`message_router.py` internals** — worth understanding in detail since every other MAVLink service builds on it:
- One `receive_loop(slot)` task per configured UAV slot, started in `start()`. Each loop drains up to 100 messages per pass via non-blocking `connection.recv_msg()`, sleeping 0 (yield-only) if messages were received or 0.01s if the queue was empty — a tight-but-cooperative poll rather than a blocking read, since `recv_msg()` must stay non-blocking for the single-reader model to coexist with everything else on the event loop.
- If the tracked connection object for a slot changes identity mid-loop (i.e. a reconnect swapped in a new `MavlinkTCPConnection`), every pending waiter for that slot is immediately failed via `cancel_slot_waiters()` — this prevents a future registered against a stale connection from hanging forever or resolving against messages from the wrong physical link.
- `dispatch(slot, message)` does two independent things per incoming message: (1) resolves any matching `_MessageWaiter` futures registered via `expect()` — matched by message-type set membership plus an optional predicate callable, and (2) invokes every long-lived handler registered for that message type (plus any `"*"` wildcard handlers) concurrently via `asyncio.gather(..., return_exceptions=True)`, logging (not raising) any handler exception so one broken handler can't take down message dispatch for the others.
- `transaction_lock(slot)` — a per-slot `asyncio.Lock` — is what `command_bridge.py` and `param_bridge.py` both wrap their request/response exchanges in, serializing e.g. an arm-command ACK-wait against a concurrent mission-upload on the same slot so their `expect()` registrations can't cross-match each other's responses.

**`interfaces.py`** defines the abstract contracts (`MAVLinkConnection`, `MAVLinkTelemetryBridge`, `MAVLinkMissionManager`, `MAVLinkCommandSender`) plus shared dataclasses (`MissionItem`, `MissionTransferResult`, `VehicleIdentity`). Its docstring frames it as a scaffold for adding real MAVLink support incrementally — most of it is now realized by the concrete classes in this directory, though `MAVLinkMissionManager`'s `clear_mission()`/`set_current_waypoint()` and `MAVLinkCommandSender`'s `takeoff()`/`return_to_launch()` signatures don't yet have concrete implementations matching them (consistent with the stub inventory below).

- **`connection.py`** — `MavlinkTCPConnection`, one instance per UAV slot, wraps `pymavlink.mavutil.mavlink_connection` over TCP (`udp:{ip}:{port}` connection string despite the class name — worth noting the naming is TCP-in-name, UDP-in-string, which reflects how MAVProxy-style endpoints are typically addressed). `connect()` runs the blocking pymavlink connect-plus-heartbeat-wait inside a thread via `asyncio.to_thread`, bounded by `asyncio.wait(..., timeout=3.0)` rather than `asyncio.wait_for` — the code comments explain why: the underlying thread can't be cancelled once started, so `wait_for`'s cancel-on-timeout would still leave the OS socket blocking for its full timeout in the background; `asyncio.wait` instead just stops awaiting and lets the orphaned thread die naturally, consuming its eventual exception silently via a done-callback so asyncio doesn't warn about it.
- **`message_router.py`** — the shared reader and the `expect()`/future-resolution mechanism that both `command_bridge.py` and `param_bridge.py` build on for correlating requests with responses without polling.
- **`telemetry_bridge.py`** (`MavlinkTelemetryBridge`) — maintains one `TelemetryPacket` dataclass per slot. `handle_message()` updates fields per MAVLink message type:

  | Message type | Fields updated |
  |---|---|
  | `HEARTBEAT` | `flight_mode` (via a custom_mode→name lookup table covering STABILIZE/ACRO/ALT_HOLD/AUTO/GUIDED/LOITER/RTL/CIRCLE/LAND/DRIFT/POSHOLD), `armed` (bit 128 of `base_mode`) |
  | `GLOBAL_POSITION_INT` | `lat`/`lon`/`altitude_m`/`relative_alt_m`, `vx`/`vy`/`vz`, `ground_speed_ms` (derived), `home_distance_m` (haversine against cached home position) |
  | `ATTITUDE` | `roll_deg`/`pitch_deg`/`yaw_deg` (radian→degree conversion) |
  | `VFR_HUD` | `air_speed_ms`, `heading_deg`, `climb_rate_ms` |
  | `SYS_STATUS` / `BATTERY_STATUS` | `battery_voltage`/`battery_current`/`battery_remaining_pct` (BATTERY_STATUS preferred when present, filters out sentinel values like `voltage==65535`) |
  | `GPS_RAW_INT` | `gps_fix`, `satellites_visible`, `hdop` |
  | `MISSION_CURRENT` / `MISSION_COUNT` | `current_waypoint`, `total_waypoints`, triggers `_update_target_waypoint()` which cross-references the locally cached mission list to compute `target_waypoint_lat/lon` and `distance_to_wp_m` |
  | `NAV_CONTROLLER_OUTPUT` | `distance_to_wp_m` (direct from autopilot, overrides the locally computed value on next message) |
  | `HOME_POSITION` | caches home lat/lon for later `home_distance_m` calculations |

  `broadcast_loop()` runs independently at `TELEMETRY_HZ` (default 5Hz), pushing whatever snapshot is currently cached per slot — it never reads the connection itself, only whatever `handle_message()` last wrote.

- **`command_bridge.py`** (`MavlinkCommandBridge`) — the actuation layer.
  - **Fully implemented:** `arm()` / `disarm()` (`MAV_CMD_COMPONENT_ARM_DISARM`, optional force-param via magic value `21196`), `set_mode()` (`MAV_CMD_DO_SET_MODE` resolved against `master.mode_mapping()`), and the complete MAVLink mission protocol — `download_mission()` (count request → per-item `MISSION_REQUEST_INT` loop with 3-retry-per-item resilience → final ACK) and `upload_mission()` (count send → request/item/ACK state machine that handles both modern `MISSION_REQUEST_INT` and legacy `MISSION_REQUEST` autopilot behavior, streaming granular progress events the whole way). All of these follow the same transaction shape: `transaction_lock(slot)` → register an ACK future via `router.expect()` → send the command → `asyncio.wait_for(ack_future, timeout=5.0)`.
  - **Stubbed** (`raise NotImplementedError("TODO: implement in control feature task")`): `return_to_launch()`, `takeoff()`, `goto()`, `handle_command_ack()`. The router layer (`app/routers/control.py`) and the frontend (`useUavControl.ts`, `controlApi.ts`) mirror these stubs exactly — UI buttons for RTL/Takeoff/Guided-Goto already exist but call straight into the throw. Full implementation plan with MAVLink primitive choices and tradeoffs is in `CONTRIBUTION_ROADMAP.md` Candidate 1.
- **`param_bridge.py`** (582 lines, fully implemented, no stubs) — the most protocol-subtle piece of the MAVLink stack.
  - **Dual encoding support.** ArduPilot and PX4 disagree on how non-float parameter values are packed into MAVLink's `PARAM_VALUE`/`PARAM_SET` float field: ArduPilot uses a plain C-style cast (an int reinterpreted as its float value, e.g. `5` → `5.0`), PX4-style bytewise targets instead reinterpret the raw bits. `encode_param_value()`/`decode_param_value()` implement both (`struct.pack`/`unpack` round-trips for the bytewise path), and `handle_message()` auto-detects which encoding a connected autopilot uses from its `AUTOPILOT_VERSION.capabilities` bitmask (checking undocumented bit flags `16`/`131072` that pymavlink 2.4.41 doesn't export as named constants — sourced from the `MAV_PROTOCOL_CAPABILITY` enum directly) or, failing that, from the `HEARTBEAT.autopilot` field matching `MAV_AUTOPILOT_ARDUPILOTMEGA`.
  - **`_fetch_all()`** drives a full `PARAM_REQUEST_LIST` sync: sends the request, then waits on an `asyncio.Event` that `handle_message()` sets on every incoming `PARAM_VALUE`, re-checking completeness (`received_count >= total`) after each wake, with up to 2 full retry passes and a 2-second inter-message timeout — tolerates an autopilot that doesn't resend the full list reliably on one request.
  - **`_retry_missing()`** follows up with individual `PARAM_REQUEST_READ` calls (by index, 2 attempts each) only for indices never received in the bulk pass — avoids re-requesting an entire parameter table (which can be 500+ entries) just to fill a handful of gaps.
  - **`set_parameter()`** sends `PARAM_SET`, waits for a matching `PARAM_VALUE` echo as confirmation; if that specific echo doesn't arrive within 2 seconds, falls back to an explicit `PARAM_REQUEST_READ` for that parameter before giving up — a resilience layer against autopilots that apply a parameter but don't always echo it promptly. Confirmation success is checked with type-aware tolerance (`_values_match`): exact integer equality for integer types, a relative+absolute floating-point tolerance for floats — not a naive `==`.
  - Progress (`param_fetch_progress`/`param_fetch_complete`/`param_set_result` events) streams over the same per-slot `/ws/control/{slot}` channel `command_bridge.py` uses for mission-upload progress, rate-limited to at most one emission per 100ms during a bulk fetch to avoid flooding the WebSocket with a message per parameter on autopilots with large parameter tables.
- **`app/schemas/control.py`** — the Pydantic contract layer both `command_bridge.py` and `param_bridge.py` sit behind at the API boundary. Beyond routine field bounds (`slot` clamped 1-4 everywhere, lat/lon range-checked), `MissionUploadRequest` carries a custom `field_validator` (`require_finite_coordinates`) that explicitly rejects `NaN`/`Infinity` in any mission-item numeric field *before* it reaches pymavlink — worth knowing about since a `NaN` silently accepted into a `MISSION_ITEM_INT` send would otherwise fail obscurely deep in the MAVLink encode/transmit path rather than as a clean 422 at the API boundary. `CommandAckResponse` is explicitly annotated as "reserved for future controls," i.e. its shape already anticipates RTL/takeoff/goto responses even though those handlers don't exist yet.
- **`apf_coordinator.py`** / **`swarm_coordinator.py`** — swarm collision-avoidance and mission-lifecycle orchestration for the copter pair. Covered in §7 (context section — actively developed by another contributor, not this document's primary focus).

### 3.4 Video pipeline (`app/services/video/`, `app/routers/video.py`)

```
UAV camera → UDP JPEG packets → VideoReceiver (background daemon thread,
    decodes to np.ndarray, stores as thread-safe .latest_frame property)
    → MultiStreamManager (re-encodes to JPEG at configured quality)
    → raw bytes broadcast over /ws/video/{port} → frontend <canvas> draw loop
```

**Wire protocol** (`receiver.py`): UDP packets are either a raw JPEG blob or optionally prefixed with a 4-byte length header — the receive loop detects which by checking whether bytes `[0:2]` are the JPEG SOI marker (`\xff\xd8`) directly, or whether it appears at offset 4 (meaning a header is present and gets stripped). Socket receive buffer is enlarged to 4MB (`SO_RCVBUF`) specifically to reduce kernel-level packet drops under bursty UDP arrival. Decode failures and empty frames are counted via a thread-safe `ReceiverStats` dataclass (frame count, drop count, rolling FPS computed over 1-second windows, average packet size, last sender address) exposed through `/api/video/status`.

`VideoReceiver` also supports a `camera` source mode (local webcam via `cv2.VideoCapture`) as an alternative to `udp`, used for local testing without a real UDP video feed.

`MultiStreamManager.ensure_stream(port, json_port)` lazily starts a `VideoReceiver` for a given port on first client connection, and — if a `json_port` is supplied — also starts a paired `UdpTelemetryReceiver` that ingests detection JSON from onboard edge-AI (this is the mechanism behind the fixed-wing UAVs' "hasVideo" detection overlay). Streams are torn down automatically once the last WebSocket client for that port disconnects (`app/routers/video.py`'s `finally` block calls `stop_stream()` if `has_video_clients(port)` is false), so idle ports don't consume resources indefinitely.

**`UdpTelemetryReceiver`** (`app/services/telemetry/udp_telemetry.py`) is a small, independent UDP JSON listener — a background daemon thread binds a socket (default port 5005, but instantiated per `json_port` by the video manager), decodes each packet as UTF-8 JSON, and stores it as `latest_data` under a `threading.Lock`, stamped with `_received_at` on arrival. It has no knowledge of video frames at all — the *pairing* between a detection JSON stream and a video stream is entirely `MultiStreamManager`'s responsibility via its `_video_to_telemetry: dict[port, json_port]` mapping.

**`MultiStreamManager._broadcast_loop(port, receiver)`** — one long-running coroutine per active video port, paced by a `next_send` monotonic-clock schedule (not a plain `sleep(interval)`, which would drift under variable processing time) to hold to `fps_limit`. Behavior worth noting:
- If nobody is watching (`ws.has_video_clients(port)` false), the loop backs off to a 0.1s poll instead of doing any encode/broadcast work — no wasted CPU on unwatched streams even while the receiver keeps running underneath.
- If the receiver has no frame yet (`latest_frame is None`), rather than sending nothing, it periodically (every `fps_limit * 2` broadcast-loop iterations) sends a generated 320×180 "NO SIGNAL" JPEG placeholder (`_make_no_signal_jpeg`) — so the frontend `<canvas>` always has *something* to render rather than a frozen last frame or blank panel when a UAV's video feed drops.
- **Detection overlay is drawn server-side, into the JPEG itself**, not left to the frontend: when a paired `json_port` has fresh detection data with `detection: true`, the loop draws a green bounding-box rectangle plus a `"Target NN.N%"` confidence label directly onto the frame via `cv2.rectangle`/`cv2.putText` before JPEG-encoding it, and always draws a small red crosshair at frame center — this happens whenever a `json_port` pairing exists, regardless of whether a target is currently detected (the crosshair is a permanent aim-reference overlay, the bounding box is conditional). The *raw* detection JSON is separately broadcast as a text WebSocket message too (`_maybe_broadcast_detections`, deduplicated by `_received_at` timestamp so the same detection isn't re-sent every video frame) — meaning the frontend receives both a pre-annotated JPEG and the raw structured detection data on the same channel, likely for a HUD layer that wants precise coordinates rather than just the baked-in pixels.
- If a `YOLODetector` instance is attached (`self._detector`), frames are also enqueued to it (`self._detector.enqueue(port, frame)`) for local inference — though per earlier project history (README/commit log), the backend-side YOLO detector was deliberately removed in favor of onboard edge-AI detection, so `self._detector` is expected to be `None` in the current architecture; this code path is effectively dormant, not deleted.

`get_latest_frame(port)` — the exact method the stitching service's live-capture endpoint calls (§4.2) — returns `receiver.latest_frame.copy()`, not the live reference: a deliberate defensive copy so a caller reading a frame for a one-off capture can't be affected by the receive thread overwriting `_latest_frame` mid-read, and so a caller mutating what it gets back (e.g. stitching's own preprocessing) can never corrupt the shared frame the video-broadcast loop is also reading.

**`VideoReceiver.latest_frame`** (a `@property` guarded by a `threading.Lock`) is the layer other subsystems read from when they need a frame without going through the WebSocket JPEG-broadcast path — this is exactly what the stitching service's live-capture feature does (§4.2). It sits *upstream* of the WebSocket re-encode step, which matters for any future video-transport migration (see `CONTRIBUTION_ROADMAP.md` Candidate 2).

Currently MJPEG-over-WebSocket — no codec, no adaptive bitrate, every frame a fully independent JPEG regardless of link quality. WebRTC migration is an unimplemented roadmap item.

### 3.5 Telemetry & System routers

- **`telemetry.py`** — `/ws/telemetry` (5Hz JSON broadcast, see §3.3), plus REST: `GET /latest` (current snapshot), `GET /sources` (configured MAVLink hosts from settings), `POST /connect` / `POST /disconnect` (per-slot connection management), `GET /status` (per-slot connected/disconnected), `GET /udp_status` (reaches into `video_manager_instance._telemetry_receivers` to report on the edge-AI JSON UDP receivers — a direct cross-router dependency worth knowing about if refactoring either router), `GET /mission/{slot}` (triggers a live `download_mission()` call against the autopilot).
- **`system.py`** — `/ws/system`, an operational log/event stream backed by an in-memory ring buffer (`_EVENT_BUFFER`, capped at 200 entries). New WebSocket connections immediately receive the last 50 buffered events as a replay before switching to live streaming — useful for an operator who reconnects mid-session and wants recent context, not just events from the moment of reconnect. REST: `GET /events?limit=N`, `GET /info` (Python version, platform, PID — basic process introspection).

### 3.6 Offline mapping (`app/routers/peta.py`)

Serves satellite tiles from a local MBTiles SQLite database (`data/peta_offline.mbtiles`, distributed via Git LFS — this file being an un-pulled LFS pointer, not a real database, was an actual bug hit and fixed earlier in this project's development). Three-tier fallback per tile request at `GET /api/peta/ubin/{z}/{x}/{y}.png`:
1. **Local MBTiles lookup** — `koordinat_vertikal_terbalik = (1 << z) - 1 - y` inverts the Y coordinate before querying, since the MBTiles spec stores tiles TMS-style (origin bottom-left) while the request comes in XYZ/Google-style (origin top-left) — a standard but easy-to-get-backwards conversion, done explicitly here rather than left implicit.
2. **Esri fallback** — on a local miss, fetches the same tile from ArcGIS Online's World Imagery service (`unduh_ubin_satelit_eksternal`, 4-second timeout, custom `User-Agent`), and — critically — writes it into the local MBTiles store (`INSERT OR REPLACE`) before returning it, so the *next* request for that same tile is served locally. This is what makes the "offline map" auto-populate over time during any session that does have connectivity, without a separate pre-seeding step.
3. **Generated placeholder** — if both the local DB and the network fetch fail (fully offline and tile never cached), synthesizes a dark 256×256 PNG with the Z/X/Y coordinates drawn as text (`buat_gambar_ubin_pengganti`) rather than returning an error or broken image — the map always renders *something* at every zoom/pan, even a cold-start fully-offline session.

All three tiers wrap their SQLite/network operations in bare `except Exception: pass` — a deliberately permissive failure mode appropriate for "never break the map," though it does mean a genuine local database corruption (distinct from a simple cache-miss) would silently fall through to the network/placeholder tiers rather than surfacing as a diagnosable error.

### 3.7 Mission control — two parallel paths

**Direct MAVLink** (primarily fixed-wing, and copters when directly reachable): via `command_bridge.upload_mission()` / `download_mission()`, the full mission protocol described in §3.3.

**Companion-bridge subprocess path** (copters, the primary documented path per `README_mission.md`) is a genuinely separate control-plane implementation from the main FastAPI MAVLink stack — it's three independent Python scripts communicating over their own UDP protocol, only loosely integrated into the web backend via subprocess invocation. Worth understanding as its own system, not just "the copter version of §3.3":

```
control_uav.py (waypoint defs, edited by hand)
        │ import
        ▼
gcs_mission_client.py (GCS/laptop — CLI, 569 lines)
        │ UDP, port from mission_config.json (default 14560)
        ▼
companion_bridge.py (Raspberry Pi on each copter — 1,094 lines)
        │ serial/UDP to flight controller
        ▼
ArduPilot Flight Controller
```

- **`control_uav.py`** (100 lines) — the single human-edited file. `build_mission()` returns a plain list of waypoint dicts in raw MAVLink mission-item shape (`command`/`frame`/`param1-4`/`x`/`y`/`z`), with inline comments documenting the MAVLink command IDs directly (`22` = `MAV_CMD_NAV_TAKEOFF`, `16` = `MAV_CMD_NAV_WAYPOINT`). Has a `if __name__ == "__main__"` self-test that pretty-prints the mission for a quick sanity check before uploading. `DEFAULT_TAKEOFF_ALT`/`HOLD_DURATION` module constants are the two values an operator is expected to tune per flight.

- **`gcs_mission_client.py`** (569 lines, runs on the operator's laptop) — a standalone MAVLink-over-UDP CLI, independent of the FastAPI backend's own MAVLink connections (it opens its own `mavutil` connection to the companion bridge). Key functions: `load_config()` (reads `mission_config.json`, the file the web UI's "Edit Connection" modal also writes to — this is the actual integration point between the two systems: the *backend* writes connection settings to a shared JSON file, and this *entirely separate* script reads them), `upload_mission()` (drip-feeds mission items with per-item retry, up to 3 attempts), `send_start_mission()` (transmits the `MAV_CMD_USER_1` go-signal, detailed below), `reload_and_upload()` (re-imports `control_uav.build_mission()` fresh and re-uploads — this is what backs the interactive `update` CLI command). Runs a background `heartbeat_sender` thread and a `status_receiver` thread that prints STATUSTEXT feedback from the companion bridge in real time, so an operator watching the terminal sees live flight-sequence progress as it happens on the Raspberry Pi.

- **`companion_bridge.py`** (1,094 lines, runs on each copter's Raspberry Pi) — the actual flight-sequencing logic. Central `BridgeState` (a lock-guarded object shared across several threads: `heartbeat_sender_gcs`, `gcs_listener_thread`, `fc_status_forwarder`, `gcs_heartbeat_watchdog`, plus the main flight-execution flow) tracks a `flight_phase` state machine — `IDLE → PRECHECK → ARMING → TAKEOFF → HOVER_STABILIZING → AUTO_RUNNING → COMPLETED` — the same phase vocabulary the newer swarm coordinator (§7) independently reimplements for its own dual-copter orchestration, suggesting the swarm state machine was modeled directly on this script's proven flow rather than designed from scratch.

  **Mission transfer protocol** (`receive_mission_from_gcs`): standard MAVLink mission download shape but with the Raspi as the *requester* — `MISSION_COUNT` received first, then the Raspi drip-feeds `MISSION_REQUEST_INT` per sequence number and waits (5s timeout) for each `MISSION_ITEM_INT`/`MISSION_ITEM` in turn, only ACKing once all items are collected. `upload_mission_to_fc()` then re-transmits that same set onward to the actual flight controller, handling both `MISSION_REQUEST_INT`/`MISSION_REQUEST` autopilot variants (`_send_item_to_fc`) — a second, independent implementation of essentially the same mission-item negotiation logic `command_bridge.py` implements for direct connections, duplicated rather than shared because this script must run standalone on a Raspberry Pi with no dependency on the main FastAPI codebase.

  **Safety gate — the core documented invariant of this entire path.** `execute_flight_sequence()` is only ever invoked after an explicit start signal; mission upload alone (`mission_uploaded_to_fc = True`) never triggers `_arm_vehicle()`. The start signal is `MAV_CMD_USER_1` (numeric value `31010`, a MAVLink user-defined command ID — not a standard command, which is why the constant is hand-declared rather than imported from `pymavlink.mavutil.mavlink`), sent as a `COMMAND_LONG` from `gcs_mission_client.py`'s `send_start_mission()` and caught in `companion_bridge.py`'s `gcs_listener_thread` → `_handle_start_command()`, which validates two preconditions before proceeding: a mission must already be uploaded to the FC, and no mission may currently be running — rejecting the start attempt with an explicit STATUSTEXT (`"[-] START ditolak: belum ada mission diterima dari GCS!"`) otherwise.

  **Flight sequence** (`execute_flight_sequence`, the main state-machine driver): arm (`_arm_vehicle`, up to 3 retries) → set mode `GUIDED` → takeoff to configured altitude → `_wait_for_altitude()` (polls relative altitude against threshold, 30s timeout) → `_wait_for_hover_stable()` (requires altitude within tolerance **and** vertical speed below threshold **continuously** for a minimum duration — `DEFAULT_HOVER_ALT_TOLERANCE=0.5m`, `DEFAULT_HOVER_VZ_THRESHOLD=0.2m/s`, `DEFAULT_HOVER_MIN_DURATION=3.0s` — a real stabilization gate before committing to autonomous mission execution, not just a fixed delay) → set mode `AUTO` → `_wait_for_wp_reached()` loop (monitors `MISSION_CURRENT`/`MISSION_ITEM_REACHED` against the last waypoint index) → `_wait_for_disarm()` once the final waypoint (the designated landing point, per `control_uav.py`'s convention of using the last waypoint as the land location) triggers autopilot auto-land/disarm.

  **Two independent failsafe mechanisms**, both defensive against losing the operator connection specifically (not the flight controller connection — that's the autopilot's own job):
  - `gcs_heartbeat_watchdog()` — if no GCS heartbeat is received for `--gcs-heartbeat-timeout` (default 3.0s) while a mission is actively running, forces the FC into `LOITER` mode (`_set_mode(fc_conn, "LOITER")`) — the vehicle holds position rather than continuing blind or falling back to a default failsafe behavior the operator didn't explicitly choose.
  - `_check_pending_mission()` — implements the **adaptive mid-flight mission update** behavior documented in `README_mission.md`: if a new mission arrives from the GCS while one is already executing, it's stored in `state.pending_mission` rather than applied immediately; only once the current mission's flight sequence reaches disarm does the pending mission get uploaded automatically, still requiring a fresh explicit start signal before it flies — so an operator can queue a mission-update without it ever silently interrupting a vehicle already in the air.

  `fc_status_forwarder()` relays flight-controller mode/armed-state changes back to the GCS as STATUSTEXT in near-real-time, which is what `gcs_mission_client.py`'s `status_receiver` thread is printing — the visible "live flight sequence" feedback loop an operator watches during a launch.

**Integration seam with the web backend.** `app/routers/control.py`'s `/api/control/companion/upload` and `/companion/start` endpoints (§3.3's sibling router) don't talk this protocol directly — they shell out to `gcs_mission_client.py` as a subprocess per request (`subprocess.Popen`, stdout streamed back via `run_in_executor` so the FastAPI event loop isn't blocked by a synchronous subprocess read), meaning every companion-mission action from the web UI pays the cost of a fresh Python interpreter start and a fresh MAVLink UDP handshake rather than reusing a persistent connection — acceptable for a low-frequency action like "upload mission" or "start flight," but worth knowing if this path is ever extended toward something latency-sensitive. `mission_config.json` (port/IP settings) is the one piece of genuinely shared, persistent state between the FastAPI backend and this otherwise-independent script family — written by the web UI's Edit Connection modal via `control.py`'s `/api/control/mission-config` endpoint, read fresh by `gcs_mission_client.py` on each subprocess invocation.

---

## 4. Image Stitching Service — Full Technical Deep-Dive

**This is the owned subsystem and the emphasis of this document.** Covers the FastAPI integration layer (`app/routers/stitching.py`, 660 lines), the stitching engine (`stitching_service/src/`, 1,359 lines across five modules), and the frontend panel (`gcs_js/src/components/stitching/`).

### 4.1 Purpose & context

Builds a single aerial orthomosaic image from a sequence of overlapping UAV-captured frames plus per-frame GPS/attitude pose data. Used for post-flight survey review, and — per the newer `app/services/mission/survey_generator.py` (swarm subsystem, §7) — is positioned as the eventual downstream consumer of dual-copter boustrophedon survey missions, meaning the frame sequences this engine will process in production are shaped by that path-generation logic, not arbitrary flight paths.

### 4.2 Integration layer (`app/routers/stitching.py`)

**Runs inside the main FastAPI process**, not as a separate microservice despite the `stitching_service/` directory name — the engine originated as a standalone script (referred to elsewhere in the repo as VISION-LIVESTITCH) and is imported via a `sys.path` insertion at router-module load time (`sys.path.insert(0, str(STITCHING_ROOT))` and `.../src`, `stitching.py:36-39`), then `from src import Combiner` / `from src import utilities as util`. This is a pragmatic integration shortcut, not a clean package import — worth knowing if the module structure ever needs reorganizing, since two different `sys.path` entries currently make both `stitching_service.src.X` and bare `src.X` / `X` import styles work simultaneously depending on which file is doing the importing (`Combiner.py` itself imports `utilities`, `geometry`, `blending` as bare top-level modules, while its own package import is `from src.redundant_filter import ...` — an inconsistency inherited from the original standalone script).

**Session model.** Each stitching run is a `StitchingSession` object keyed by a regex-validated `sessionId` (`^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$`), backed by a real filesystem directory: `stitching_service/sessions/{id}/images/` (input frames) and `.../output/` (final mosaic + intermediate results). Sessions are **rediscoverable from disk on backend restart** (`discover_existing_sessions()`, scans `SESSIONS_ROOT` for directories matching the ID pattern with an `images/` subfolder and reconstructs session state) — so an in-progress or completed stitching session survives a backend reload, which matters operationally since the backend also serves the live video/telemetry pipeline and might get restarted for unrelated reasons during a field session.

**Concurrency control.** `StitchingSession.claim_stitch()` / `finish_stitch()` use a `threading.Lock` to guarantee only one stitch operation runs per session at a time — a second trigger while one is in-flight is rejected (`{"status": "Stitching already in progress"}`) rather than queued or run concurrently, since the underlying `Combiner` maintains mutable running state (`self.result_image`, `self.H_global_prev`) that isn't safe for concurrent stitch calls against the same session.

#### Frame ingestion — two independent paths

**1. File upload** — `POST /api/stitching/session/{id}/upload`, plain multipart form (`UploadFile` list), streams each file to disk in 1MB chunks, validates extension against `IMAGE_SUFFIXES` (`.jpg/.jpeg/.png/.tif/.tiff`) before accepting. Zero dependency on the video pipeline — this path works for post-flight batch imagery regardless of how it was captured (SD card pull, manual transfer, etc.).

**2. Live capture** — `POST /api/stitching/session/{id}/capture-stream`. Request body (`StreamCaptureRequest`) carries `streamPort`, optional `jsonPort`, optional `uavId`, and `jpegQuality` (default 95, deliberately higher than the video-broadcast default of 80, since this copy is meant for stitching-quality feature detection, not just human viewing). Flow:
   1. `video_manager_instance.ensure_stream(stream_port, json_port)` — lazily starts the `VideoReceiver` for that port if not already running (piggybacking on the same manager the live video panel uses).
   2. `wait_for_stream_frame()` polls `video_manager_instance.get_latest_frame(stream_port)` — which reads straight through to `VideoReceiver.latest_frame` (§3.4) — every 50ms up to a 2-second deadline, since a freshly-started receiver needs a moment to receive its first UDP packet.
   3. On success, `write_stream_frame()` names the file deterministically and collision-resistantly: `{timestamp}_{milliseconds:03d}_{unique_id:09d}_{source_name}_{stream_port}.jpg`, where `unique_id` is `time.time_ns() % 1_000_000_000` — astronomically unlikely to collide even under rapid repeated captures — and writes it via `cv2.imwrite` at the requested JPEG quality.
   4. `record_new_images()` is called, which both notifies WebSocket clients of the new file and checks the auto-stitch threshold (below).

   Returns `409 Conflict` if no frame has arrived within the timeout (i.e., nothing is actually streaming on that port yet) — this is the one behavior a caller needs to handle explicitly, since it's a legitimate "not ready yet" state rather than a hard failure.

**Auto-stitch triggering.** A `watchdog.observers.Observer` filesystem watcher (`SessionFolderHandler`) can be attached per-session to its `images/` folder. On `on_created` for any image-suffixed file: debounces repeated events for the same path (1-second window), waits for the file size to stabilize across up to 6 polls at 0.5s intervals (`_wait_until_stable`, guards against triggering on a partially-written file), then — if `auto_stitch_enabled` and the image count since the last stitch has crossed `auto_stitch_threshold` (default 5, configurable per session) — atomically claims and launches a stitch via `claim_stitch()` + `schedule_from_thread(run_stitching(...))`. `schedule_from_thread` bridges the watchdog observer's own OS thread back onto FastAPI's asyncio event loop via `asyncio.run_coroutine_threadsafe`, using a `service_loop` reference captured once at `startup()`.

Note the file-upload path and the folder-watcher's own notification path are deliberately *not* both firing for the same event — `record_new_images()` only sends `file_detected` WebSocket notifications itself when folder monitoring is **off** (`if not session.config.folder_monitoring_enabled: ...`), since when monitoring is on, the watchdog handler's `on_created` callback already sends that notification — avoiding duplicate events for the same file.

#### Stitch execution & result retrieval

`trigger_stitch()` / auto-trigger both funnel into `run_stitching()`, which runs the actual (CPU-bound, blocking) `create_mosaic()` call inside `asyncio.to_thread` so it doesn't block the event loop that's simultaneously serving video/telemetry/control traffic — an important detail, since stitching a long image sequence can take significant wall-clock time and the backend has other latency-sensitive duties running concurrently. Progress is entirely event-driven over `/ws/stitching/{session_id}`: `stitching_started` (with image count) → `stitching_completed` (success flag, elapsed time, error message if failed, and a direct URL to fetch the result if successful).

Result retrieval (`GET .../result`) has a graceful fallback: if the named output file (`output_name`, default `finalResult.png`) doesn't exist yet (e.g. a stitch is still running or crashed partway), it falls back to the most recently modified `intermediateResult_*.png` — meaning the frontend can always show *something* rather than a hard 404 while a long stitch is in progress. `Cache-Control: no-store` is set explicitly so the browser never serves a stale mosaic after a re-stitch.

Full REST surface: `GET /`, `GET /health`, `GET /sessions`, `POST /session/create`, `POST /session/{id}/toggle-monitoring`, `POST /session/{id}/toggle-auto-stitch`, `POST /session/{id}/upload`, `POST /session/{id}/capture-stream`, `POST /session/{id}/stitch`, `GET /session/{id}/result`, `GET /session/{id}/intermediates`, `GET /session/{id}/intermediates/{file_name}`, `GET /session/{id}/status`, plus `WS /ws/stitching/{session_id}`.

### 4.3 The stitching engine (`stitching_service/src/`)

| File | Lines | Role |
|---|---|---|
| `Combiner.py` | 360 | Core sequential-stitching orchestrator — the main algorithm, actively used |
| `geometry.py` | 55 | Pose-based image unrotation, perspective warp with padding — actively used |
| `blending.py` | 356 | Six compositing strategies — only one (`ROIfeatherBlender`) is actually wired in |
| `redundant_filter.py` | 341 | Two full, paper-cited redundant-frame-selection algorithms — imported, never invoked |
| `utilities.py` | 248 | EXIF/GPS metadata extraction, image directory import, legacy match-drawing helper |
| `preprocessing.py` | 0 | Empty file — dead placeholder |

#### 4.3.1 Algorithm: `Combiner.create_mosaic()` — sequential pairwise chaining

Not a batch/global stitcher. Images are combined **one at a time, strictly in capture order** — each new frame is matched only against the *running composite so far*, never against all prior frames or any global reference. `create_mosaic()` simply loops `combine(i)` for `i in range(1, len(image_list))`, seeded by `image_list[0]` as the initial mosaic.

**Construction-time preprocessing** (`__preprocess_images`, runs once over the whole input set before any stitching begins): each raw frame is downsampled by a fixed factor of 5 via nearest-neighbor strided slicing (`img[::5, ::5]` — not an area-averaged resize), then unrotated via `geometry.computeUnRotMatrix(pose)`. That function builds a rotation matrix from the frame's `[X, Y, Z, Yaw, Pitch, Roll]` pose vector (per the classical approach at planning.cs.uiuc.edu/node102.html, cited directly in the docstring) and applies it via `warpPerspectiveWithPadding` — so every frame entering the stitching loop has already had its aircraft-attitude-induced perspective distortion removed, approximating a straight-down nadir view regardless of the actual roll/pitch at capture time. Processed frames replace raw ones in-place with explicit `gc.collect()` every 5 images to bound peak memory on long sequences.

**Per-step pipeline** (`combine(index)`, matching `image_list[index-1]` — the running mosaic's most recent contributor — against `image_list[index]`):

1. **ROI prediction** (`__predict_roi`): if a previous relative homography (`H_rel_prev`) exists, the incoming frame is cropped to the region the last transform predicts will actually overlap, plus a 15% margin, before feature detection — this is a real performance optimization (fewer pixels to run SIFT over) but rests on an explicitly-documented assumption: *"camera motion is mostly forward and the scene is mostly planar."* If no prior homography exists (first pairing), the full frame is used.
2. **Feature detection** (`__detect_features`): `cv2.SIFT_create(1800)` — up to 1800 keypoints — run on both the previous frame (full) and the new frame (ROI-cropped, if applicable). Keypoints from a cropped region have their pixel coordinates translated back into full-image space by manually constructing new `cv2.KeyPoint` objects with adjusted `.pt`.
3. **Matching** (`__match_features`): `cv2.BFMatcher()` (brute-force, not FLANN) `.knnMatch(k=2)`, filtered by Lowe's ratio test at a 0.55 threshold (tighter than the conventional 0.7-0.8 range often used — a stricter-than-typical match acceptance criterion, favoring precision over recall).
4. **Transform estimation** (`__estimate_transform`): tries `cv2.estimateAffinePartial2D` first (4 degrees of freedom — rotation, uniform scale, translation); if that returns `None`, falls back to full `cv2.findHomography(..., method=cv2.RANSAC)` (8 DoF, handles perspective distortion the affine model can't).
5. **Homography chaining**: the newly estimated relative transform is right-multiplied onto the running global transform (`H_global_current = H_global_prev @ H_rel`), then renormalized by its `[2,2]` element to keep the homogeneous scale consistent across the whole chain.
6. **Canvas computation & warping** (`__compute_canvas_bounds`, `_warp_images`): the bounding box needed to contain *both* the existing mosaic and the newly-transformed frame is computed fresh each step, and **both the entire accumulated mosaic and the new frame** are warped onto that new canvas via `cv2.warpPerspective` — meaning the full existing mosaic gets re-rendered every single step, not just the new content.
7. **Blending** (`ROIfeatherBlender._roi_feather_blend`, detailed in §4.3.2): composites the two warped images at their overlap seam.
8. **Persistence & diagnostics**: writes `intermediateResult_{index}.png` and a `matches_{index-1}_{index}.jpg` match-visualization to disk **unconditionally, every step**, and prints detailed per-stage timing (`⏱️` prefixed) throughout — the whole class maintains a `timing_stats` dict (preprocessing/feature_detection/matching/transformation/warping/blending/total) and prints a full breakdown table via `_print_timing_summary()` at the end of `create_mosaic()`, also persisted to `timing_stats.txt` in the output folder.

**Self-documented limitation.** The source contains an explicit comment acknowledging the core architectural tradeoff: *"Attempt to combine one pair of images at each step. Assume the order in which the images are given is the best order. This introduces drift!"* (`Combiner.py:203-204`). Pure pairwise chaining with no loop closure or global bundle adjustment means homography error compounds monotonically over a long sequence — the longer the survey run, the more the accumulated transform can diverge from ground truth, even though every individual per-step estimate might be locally accurate.

**Cost characteristic.** Because the entire running mosaic (not just the new frame) is re-warped every step, and the canvas keeps growing as coverage expands, per-step warp cost increases over the course of a run — total warp cost across a full sequence of N frames grows faster than linearly in N, which is the classic pitfall of naive growing-panorama implementations.

#### 4.3.2 Blending strategies (`blending.py`) — six implementations, one in production

| Class / function | Approach | Status |
|---|---|---|
| `PyramidBlender` | Multi-band Laplacian pyramid blending (Gaussian pyramid → Laplacian pyramid → per-level mask blend → reconstruct), with three variants: full-resolution (`blend_images`), downsampled-then-upsampled for speed (`blend_images_lowres`, uses `INTER_AREA`/`INTER_CUBIC`), and ROI-cropped for efficiency (`blend_images_roi`) | Implemented, **not called** by `Combiner` |
| `simpleBlender.feather_blend` | Single distance-transform-based feather, flat (not multi-band) | Implemented, **not called** |
| `ROIfeatherBlender._roi_feather_blend` | Computes binary masks for each warped image, fills non-overlap regions directly, then in the overlap bounding box computes per-pixel alpha from the *ratio* of each image's distance-transform value (`dist1 / (dist1+dist2)`, raised to the 3rd power to sharpen the transition), blends only within that ROI | **Active — the only blend strategy actually used**, called at `Combiner.py:299` |
| `hybridBlender.hybrid_blend` | Adaptive: fast feather blend for most of the frame, with a `fast_mode` toggle to fall back to full `PyramidBlender` for quality-critical cases | Implemented, **not called** |
| `AdaptiveWeightedFusion` | Gaussian center-weighted map (`generate_weight_map`) optionally combined with Sobel-gradient-based edge weighting (`adaptive_weight_map`) — biases blend weight toward image centers and away from soft/blurry regions | Implemented, **not called** (note: `adaptive_weight_map` calls `AdaptiveWeightedFusion.gaussian_weight_map`, a method name that doesn't match the actual defined method `generate_weight_map` — this specific function path appears to have a latent bug and has likely never been exercised) |
| `incrementalFusion` | Stateful accumulator across a whole canvas using a winner-take-all rule per pixel (`weight_map > existing_weight`) — a different architectural model entirely (running weighted composite rather than pairwise sequential blend) | Implemented, **not called**, and architecturally incompatible with `Combiner`'s current pairwise-warp loop without restructuring |

The active production path (`ROIfeatherBlender`) is the simplest of the six — a single feather blend confined to the overlap bounding box, cubed distance-ratio alpha. It does not perform any exposure compensation (a `compensate_exposure()` helper function exists at the bottom of `blending.py` but is also unused/uncalled) or multi-band frequency separation, meaning visible seams from lighting/exposure mismatches between frames are a plausible artifact in the current production output that the already-implemented `hybridBlender`/`PyramidBlender` paths were built to address but have never been evaluated against real output.

#### 4.3.3 Redundant-frame filtering (`redundant_filter.py`) — implemented, never invoked

Two independent, directly paper-cited algorithms live in this file, imported at `Combiner.py:9` (`from src.redundant_filter import keyframe_selection_yuan2024, redundancy_removal_he2024`) but **called nowhere** in the active pipeline — every ingested frame gets stitched regardless of redundancy.

**Paper 1 — He et al., *Drones* 2024** (`redundancy_removal_he2024`): computes a simplified BRISQUE (Blind/Referenceless Image Spatial Quality Evaluator) proxy score per frame (`compute_brisque_score`) — MSCN (Mean Subtracted Contrast Normalized) coefficients via local Gaussian mean/variance, asymmetric distribution variance between left/right MSCN tails, and pairwise-neighbor structural-distortion statistics in four directions (horizontal/vertical/two diagonals) — combined into a single score where **lower = better quality**. Frames are grouped into fixed-size intervals (default 3), and within each interval only the single lowest-score (best-quality) frame is retained, discarding the rest as redundant. This is a quality-based *and* redundancy-reducing filter simultaneously: it doesn't just thin out near-duplicates, it prefers the sharpest/least-distorted frame among near-duplicates.

**Paper 2 — Yuan et al., SCIRP 2024** (`keyframe_selection_yuan2024`): a genuinely more sophisticated two-stage algorithm.
- *Stage 1 — overlap-rate curve fitting*: from the current keyframe, samples 4 candidate frames ahead within a search window (default max 300 frames, sampled at ~75-frame intervals), computes pairwise overlap rate at each sample point via real feature-matching + homography + warped-corner intersection area (`compute_overlap_rate`), then fits a **Lagrange polynomial interpolation** (`lagrange_interpolation`) through those sample points to estimate the overlap-rate curve continuously, and scans forward along that fitted curve to find the furthest frame still above an 80% overlap threshold — that becomes the *candidate* next keyframe.
- *Stage 2 — remapping-error validation*: computes the actual mean reprojection error (`compute_remapping_error`, real SIFT/BFMatcher/homography once more, not the fitted estimate) between the current keyframe and the candidate; if that error is within a 4-pixel threshold, the candidate is accepted outright. If not, it searches **backward** from the candidate toward the current keyframe for the first frame that does satisfy the error threshold, accepting that instead — and only falls through to a forced single-frame advance if nothing in the backward search passes.

Both algorithms reduce N (the number of frames actually fed into `Combiner`) *before* stitching begins, which is the highest-leverage lever available for improving both stitching speed and drift accumulation, since every downstream cost (SIFT detection, matching, warping, and error compounding) scales with frame count. Wiring one of these in is the top-priority item in `CONTRIBUTION_ROADMAP.md`'s stitching refactor plan.

`utilities.py` additionally contains its own, much simpler `redundancy_filter()` (pure GPS-distance-threshold based, no image content involved) — a third, even more basic redundancy heuristic present in the codebase, also unused.

#### 4.3.4 Metadata extraction (`utilities.py`)

Two independent EXIF-GPS extraction implementations coexist: `extract_gps_data_PIL` (via `PIL.Image.getexif()` + `PIL.ExifTags`) and `extract_gps_data` (via the `exifread` library, processing raw EXIF rational-number tag structures directly) — both convert DMS (degrees/minutes/seconds) GPS tags to decimal degrees, handling N/S and E/W reference sign flipping. `importData()` is the actual entry point used by the router (`create_mosaic()` in `stitching.py` calls `util.importData(str(session.image_folder), return_as_dict=True)`), which walks a directory, loads every recognized image extension via `cv2.imread`, and pairs each with its extracted GPS metadata — falling back to `(0.0, 0.0, 0.0)` with a printed warning if a frame has no usable EXIF GPS data at all, rather than failing the whole batch.

### 4.4 Frontend (`gcs_js/src/components/stitching/`, `app/stitching/page.tsx`)

**Fully wired end-to-end — functionally complete, not a scaffold.** Traced the actual data flow through `StitchingPanel.tsx` (314 lines, the orchestrating parent component):

- On mount, loads the session list (`listSessions()`) and auto-selects the first one.
- Once a session is selected, polls `getSessionStatus()` + `listIntermediates()` every 3 seconds (`setInterval`) in addition to reacting to live WebSocket events — a belt-and-suspenders approach that keeps the UI accurate even if a WebSocket event is somehow missed.
- `useStitchWebSocket` hook maintains the live `/ws/stitching/{session_id}` connection; incoming events are pushed into a capped 100-entry rolling event log (`StitchingEventLog.tsx`) and drive user-facing notices/errors (e.g. `stitching_completed` with `success: true` shows a formatted elapsed-time notice; `success: false` surfaces `error_message` directly).
- `streamSources` is computed reactively from `useGCSStore()`'s configured UAVs, filtered to only those with `hasVideo: true` and a valid configured `streamPort` — meaning the "capture from live stream" UI option only appears for UAVs that are actually video-capable and configured (naturally excludes copter slots 3/4 per the fleet topology in §2).
- `StitchingSessionControl.tsx` (276 lines) — session creation form, auto-stitch threshold configuration, folder-monitoring toggle, file upload, and the live-capture trigger per available stream source.
- `StitchingResultViewer.tsx` (119 lines) — displays the current mosaic via `getResultImageUrl(sessionId, resultRevision)`, where `resultRevision` is a locally-incremented counter used purely as a cache-busting query param (`?v=N`) so the browser doesn't serve a stale cached image after a re-stitch — paired with the backend's own `Cache-Control: no-store` header for belt-and-suspenders freshness.
- `StitchingEventLog.tsx` (82 lines) — renders the rolling event log plus the WebSocket connection state.
- `lib/stitchApi.ts` (99 lines) — clean 1:1 wrapper over every backend endpoint, with a typed `StitchApiError` that surfaces the backend's `detail`/`error` payload fields directly to the UI rather than a generic failure message.

**Assessment:** contrary to an earlier impression that this frontend layer was only partially complete, direct code tracing shows it's functionally complete — every backend capability (session lifecycle, both ingestion paths, monitoring toggles, manual/auto stitch trigger, result + intermediate viewing, live event log) has a corresponding wired-up UI path with no stub `TODO` markers found anywhere in these five files. Any remaining "half-done" feeling is more likely about visual polish/design-system alignment (see `CONTRIBUTION_ROADMAP.md`'s resolved design-system section — style this panel against `SOFT_DESIGN_SYSTEM.md`'s live tokens) than missing functionality — worth a fresh visual pass against the current `globals.css` rather than assuming logic work remains.

### 4.5 Known limitations & the optimization roadmap

Summarized here for completeness; the full actionable, prioritized, effort-estimated version — with suggested sequencing and explicit tradeoffs for each item — lives in `CONTRIBUTION_ROADMAP.md` under **"Owned Work — Image Stitching Algorithm Refactor & Optimization."**

1. Redundancy/keyframe filtering exists (§4.3.3) but is never invoked — highest-leverage fix, reduces N directly.
2. Features recomputed twice per frame (each frame is detected once as "new" and again as "previous" the following step) — cacheable for a ~2x feature-detection speedup at zero algorithmic cost.
3. Full-mosaic re-warp every step on a continuously growing canvas — superlinear cost growth, the core "naive growing panorama" trap.
4. No global optimization/loop-closure — drift is self-acknowledged in source, uncorrected.
5. Five of six implemented blending strategies are dead code; only the simplest (no exposure compensation, no multi-band frequency separation) runs in production.
6. Naive nearest-neighbor downsampling (`img[::5,::5]`) instead of area-averaged resize — free quality improvement available.
7. Zero parallelism anywhere in the pipeline, despite feature detection being embarrassingly parallel across independent frames.
8. Unconditional per-step disk I/O (intermediate PNG + match-visualization JPEG) inside the hot loop, every single step, regardless of debug need.
9. The forward-motion ROI-prediction assumption is a real correctness risk against the swarm subsystem's boustrophedon survey paths (§7), which include 180° turns where that assumption breaks down.
10. `preprocessing.py` is a dead empty file — undecided whether it's a stale placeholder or the intended home for logic currently inline in `Combiner`.

---

## 5. Frontend Architecture (`gcs_js/`)

Next.js 15 (App Router) + React 19. Pages under `src/app/`: `/` (main dashboard — video panels, telemetry, map), `/mission`, `/params`, `/fulldata`, `/stitching`, `/swarm`, `/logging` — all sharing a common header (`TopBar.tsx`) and UAV-selector chrome.

**State management.** `useGCSStore.tsx` is a React Context (despite the "Store" name — not Redux, not Zustand) holding: per-UAV connection configs (`UAVConnectionConfig`), per-UAV metric display layout (`MetricConfig[]`, defaults defined inline covering altitude/speed/heading/satellites/HDOP/waypoint-distance/home-distance), theme (`ThemeMode`), and — as of `ad7f843` — swarm survey path state. All UAV/metric config is persisted to `localStorage` under `bima_gcs_uav_{id}` / `bima_gcs_uav_metrics_{id}` keys, hydrated post-mount via `queueMicrotask` specifically to avoid Next.js SSR hydration mismatches (server-rendered markup can't know `localStorage` contents, so the hydration is deliberately deferred to a microtask after mount).

**Data hooks.** `useWebSocket.ts` (generic WS client used by multiple panels), `useUavControl.ts` (thin wrapper over `lib/controlApi.ts` — mirrors the backend's arm/disarm/mode as fully implemented and RTL/takeoff/goto as stubs, `TODO_MESSAGE` throw on both client and server), `useStitchWebSocket.ts` (dedicated to the stitching session channel, §4.4).

**Design system.** `SOFT_DESIGN_SYSTEM.md` is confirmed live — verified directly against shipped source, not assumed: `globals.css:20/120` neutral tokens (`--bg-base: #0E0E0E` dark / `#FAFAF8` light), `globals.css:78-82` radius scale (6/10/14/18px), `layout.tsx:2,8,13` font stack (`Inter` + `JetBrains_Mono` via `next/font/google`). A second, conflicting spec (`DESIGN_SYSTEM.md`, a near-black motorsport-cockpit aesthetic with square 2px corners and Barlow Condensed/IBM Plex Mono) exists in the same directory but was never actually shipped — despite containing a detailed "post-render critique" section that reads as if it were the implemented direction, it's historical/superseded. Any new component work, including further stitching-panel styling, should reference `SOFT_DESIGN_SYSTEM.md` and the live `globals.css` token names.

---

## 6. Cross-Cutting Concerns

- **No authentication anywhere.** Every REST and WebSocket endpoint — arm/disarm/mission-upload, the swarm start/abort, and every stitching-session action — is reachable by anything that can reach the host or Tailscale network. `allow_origins=["*"]` + `allow_credentials=True` is flagged in-source as needing production hardening. Full gap analysis and a pragmatic shared-token v1 proposal in `CONTRIBUTION_ROADMAP.md` Candidate 3.
- **No automated test coverage for the video, telemetry, or stitching pipelines.** `tests/` contains exactly one file, `test_mavlink_control.py`. Manual/SITL verification is the documented practice (`README_mission.md`'s SITL instructions using `--fc-connection udp:127.0.0.1:14551`).
- **Two independent logging systems.** Standard Python `logging` to `logs/ground_station.log` (rotating, 10MB × 5 backups) covers general application/error logging. A separate, newer JSONL-based flight-session logger (`app/services/data_logger.py`, introduced in `ad7f843`) records 1Hz telemetry snapshots per swarm mission session, independent of the general application log — these serve different purposes (operational debugging vs. flight-data record-keeping) and shouldn't be conflated.
- **Configuration** is environment-variable driven via `pydantic-settings` (`app/config/settings.py`), `.env`-file based, and includes Tailscale-aware IP self-resolution (`get_tailscale_ip()` in `main.py`, tries the Tailscale MagicDNS gateway `100.100.100.100` first, falls back to general internet routing via `8.8.8.8` to find the primary LAN IP) — the mechanism that lets the decoupled Next.js frontend find the backend across a remote-deployment network without hardcoded addresses.

---

## 7. Swarm Coordination Subsystem (context only)

Introduced in commit `ad7f843`, actively developed by another contributor — summarized here purely for system-level context, since the swarm survey-path generator is upstream of what the stitching service will eventually process, and this document's author should not modify these files.

- **`apf_coordinator.py`** — Artificial Potential Field collision-avoidance for the copter pair (slots 3/4 only, hardcoded): attractive force toward each UAV's active waypoint, repulsive force from peer proximity and predicted closest-point-of-approach, hysteresis state machine (INACTIVE→ACTIVE→COOLDOWN), priority-based right-of-way between the two copters, head-on vertical separation. Pure function, no I/O. Self-documented in-source as SITL-tuned constants, explicitly flagged as not yet safe for real flight without field validation.
- **`swarm_coordinator.py`** (935 lines) — full mission lifecycle state machine (IDLE→UPLOADING→READY_TO_START→STARTING→ARMING→TAKEOFF→HOVER_STABILIZING→AUTO_RUNNING→COMPLETED, plus ABORTING/FAILED). Abort reuses `command_bridge.set_mode()`/`disarm()` (LOITER if airborne, disarm only if grounded — a real safety check, never disarms mid-flight). Mission upload/start goes through the same `mission_scripts/gcs_mission_client.py` subprocess path as manual companion-bridge missions (§3.7) rather than `command_bridge.upload_mission()`. APF corrections are streamed directly to the raw pymavlink connection at 5Hz via a backdoor accessor (`command_bridge._require_master()`), bypassing the command bridge's normal transaction machinery.
- **`survey_generator.py`** — shapely-based boustrophedon/lawnmower path generator, splits one operator-drawn polygon into two per-UAV sweep patterns with full WGS84↔ENU coordinate round-tripping. **Directly relevant to stitching:** generated paths include 180° turns between sweep lines, at which point the stitching engine's forward-motion ROI-prediction assumption (§4.3.1, point 9 in §4.5) can break down — flagged as a genuine cross-feature correctness risk worth raising with whoever owns this file.
- **`conflict_checker.py`** — offline pre-flight trajectory simulation, interpolates both generated paths over time and flags any 1-second sample where separation drops below threshold, before upload.
- **API** (`/api/swarm/*`): status, upload, start (requires literal string `"CONFIRM_START"` double-confirmation), abort, reset, generate.
- **Frontend:** `SwarmControlPanel.tsx` (682 lines) drives the full lifecycle; `SwarmSurveyMap.tsx` renders generated paths + predicted conflict markers. Both fully wired, not scaffolding.

---

## 8. Reference Map

**Backend Python** (excluding `venv`, `dump_trash`) — `app/config/settings.py`, `app/main.py`, `app/routers/{control,logging,peta,stitching,swarm,system,telemetry,video}.py`, `app/schemas/{control,swarm,swarm_mission}.py`, `app/services/data_logger.py`, `app/services/mavlink/{apf_coordinator,command_bridge,connection,interfaces,message_router,param_bridge,swarm_coordinator,telemetry_bridge}.py`, `app/services/mission/{conflict_checker,survey_generator}.py`, `app/services/telemetry/{generator,udp_telemetry}.py`, `app/services/video/{manager,receiver}.py`, `app/services/websocket/manager.py`, `mission_scripts/{companion_bridge,control_uav,gcs_mission_client}.py`, `stitching_service/src/{blending,Combiner,geometry,preprocessing,redundant_filter,utilities}.py`.

**Frontend TypeScript** — `gcs_js/src/app/{page,layout,mission/page,params/page,fulldata/page,stitching/page,swarm/page,logging/page}.tsx`, `gcs_js/src/components/{control,header,map,modal,stitching,telemetry,video}/*.tsx`, `gcs_js/src/hooks/*.ts(x)`, `gcs_js/src/lib/*.ts`, `gcs_js/src/config/*.ts`, `gcs_js/src/types/*.ts`.

**Related documents in this repo:** `README.md` (Indonesian, user-facing setup guide), `README_mission.md` (mission-control 3-script protocol), `CONTRIBUTION_ROADMAP.md` (actionable candidate work items — flight command completion, WebRTC, auth — plus the full prioritized stitching refactor plan with effort estimates), `gcs_js/SOFT_DESIGN_SYSTEM.md` (live design tokens, confirmed against shipped code), `gcs_js/DESIGN_SYSTEM.md` (superseded, historical only).
