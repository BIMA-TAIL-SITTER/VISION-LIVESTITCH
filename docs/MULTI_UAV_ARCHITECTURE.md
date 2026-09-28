# Multi-UAV System Architecture

Extends the single-UAV architecture in [SYSTEM.md](./SYSTEM.md) to concurrent multi-UAV ingestion. Renamed components:

| Old name (SYSTEM.md) | New name |
|---|---|
| Core Engine: `Combiner.py` | **Mapping Engine** |
| `service.py` FastAPI | **Stitching Modular Service** |
| Session Directory | **Session** (monitors intermediate stitching results) |

```mermaid
graph LR
    subgraph UAV1[UAV 1 Pipeline]
        direction LR
        A1[Camera Module] --> B1[Image Sender Program]
        B1 -->|TCP :5001| C1[receiver_socket.py]
        C1 -->|Write .jpg| D1[(Session: uav_1)]
    end

    subgraph UAV2[UAV 2 Pipeline]
        direction LR
        A2[Camera Module] --> B2[Image Sender Program]
        B2 -->|TCP :5002| C2[receiver_socket.py]
        C2 -->|Write .jpg| D2[(Session: uav_2)]
    end

    subgraph UAV3[UAV 3 Pipeline]
        direction LR
        A3[Camera Module] --> B3[Image Sender Program]
        B3 -->|TCP :5003| C3[receiver_socket.py]
        C3 -->|Write .jpg| D3[(Session: uav_3)]
    end

    subgraph SVC[Stitching Modular Service - GCS Backend]
        direction LR
        E[Watchdog + Auto-Stitch Trigger]
        F1((Mapping Engine: uav_1))
        F2((Mapping Engine: uav_2))
        F3((Mapping Engine: uav_3))
        E --> F1
        E --> F2
        E --> F3
    end

    D1 <-->|Watchdog Event / Read-Write| F1
    D2 <-->|Watchdog Event / Read-Write| F2
    D3 <-->|Watchdog Event / Read-Write| F3

    G[GCS Dashboard] <-->|REST / WebSockets per session_id| E
```

## Notes

- Each UAV keeps its own TCP port, `asyncio` connection handler, and isolated **Session** — one UAV's ingestion or stitching load never blocks another's (see [SOCKET.md](./SOCKET.md) for why `asyncio` replaced the old blocking `recvall` approach).
- **Mapping Engine** lives entirely on the GCS backend side, inside the **Stitching Modular Service** — it is not deployed on the UAV. The service's Watchdog + auto-stitch trigger dispatches one Mapping Engine instance per session; the Mapping Engine itself is a backend component invoked by the service, not a standalone microservice.
- The GCS Dashboard multiplexes over `session_id` (`uav_1`, `uav_2`, `uav_3`) against the same REST/WebSocket surface described in [TECHNICAL.md](./TECHNICAL.md#6-api-reference-servicepy).
- Cross-UAV mosaic merging (combining corridor strips into one map) is still not implemented — each Mapping Engine instance stitches its own session independently, per the limitation noted in TECHNICAL.md §10.5.
