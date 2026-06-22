# receiver_socket.py
import socket
import struct
import os
import time
import argparse
from pathlib import Path
import asyncio

HOST = "0.0.0.0"  # listen di semua interface
# PORT = 5001       # bebas, asal sama dengan sender

# # for multi UAV support, we can enable more tcp ports and run multiple instances of this server, each with a different session_id and port.
# PORT_ONE = 5001
# PORT_TWO = 5002
# PORT_THREE = 5003

# for multi uav support configuration
UAV_CONFIG = {
    "uav1": {"port": 5001, "session_id": "uav_1"},
    "uav2": {"port": 5002, "session_id": "uav_2"},
    # "uav3": {"port": 5003, "session_id": "uav_3"},
}

async def handle_uavs_connections(reader, writer, session_id, save_dir):
    "async handler for each UAV connection, to be used with asyncio.start_server"
    write_addr = writer.get_extra_info('peername')
    print(f"[SERVER] Connected from {write_addr} for session {session_id}")
    img_counter = 0
    try:
        while True:
            # 1) baca 4 byte panjang data
            header = await reader.readexactly(4)
            if not header:
                print(f"[SERVER] Client {write_addr} disconnected (no header).")
                break

            # 2) unpack jadi integer (big-endian)
            length = struct.unpack("!I", header)[0]
            if length == 0:
                print(f"[SERVER] Got length 0 from {write_addr}, skip.")
                continue

            # 3) baca data gambar sepanjang length
            img_data = await reader.readexactly(length)
            if img_data is None:
                print(f"[SERVER] Client {write_addr} disconnected (no data).")
                break

            # 4) buat nama file: timestamp + counter
            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"{ts}_{img_counter:04d}.jpg"
            save_path = save_dir / fname

            with open(save_path, "wb") as f:
                f.write(img_data)

            print(f"[SERVER] Saved from {write_addr}: {save_path} ({len(img_data)} bytes)")
            img_counter += 1

    except asyncio.IncompleteReadError:
        print(f"[SERVER] Client {write_addr} disconnected unexpectedly.")
    except Exception as e:
        print(f"[SERVER] Error with client {write_addr}: {e}")
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"[SERVER] Connection with {write_addr} closed.")
        print(f"[session {session_id.upper()}] Total images received: {img_counter}")


async def main():
    print("[SERVER] Starting server MULTI UAV IMAGE RECEIVER...")
    servers = []
    for uav_name, config in UAV_CONFIG.items():
        port = config["port"]
        session_id = config["session_id"]
        save_dir = Path(f"sessions/{session_id}/images")
        save_dir.mkdir(parents=True, exist_ok=True)

        server = await asyncio.start_server(
            lambda r, w, sid=session_id, sd=save_dir: handle_uavs_connections(r, w, sid, sd),
            host=HOST,
            port=port
        )
        servers.append(server)
        print(f"[*] Listening {session_id.upper()} pada port {port}")

    try:
        await asyncio.gather(*[server.serve_forever() for server in servers])
    except KeyboardInterrupt:
        print("[SERVER] Server stopped by user.")
    finally:
        for server in servers:
            server.close()
            await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[SERVER] Server stopped by user.")