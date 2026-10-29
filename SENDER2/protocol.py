"""MTU-sized UDP fragments with bounded, source-isolated reassembly.

CRC32 detects corruption, not sender authentication. Use a trusted/private link.
No retransmission: missing previews expire; the onboard originals remain intact.
"""
from collections import Counter, OrderedDict
import json
import struct
import time
from uuid import UUID
import zlib
from .models import FrameMeta

HEADER = struct.Struct("!4sB32s32s16sQHHII")
MAGIC = b"VS2F"
MAX_FRAME_BYTES = 8_000_000
MAX_METADATA_BYTES = 16384
MAX_CHUNKS = 8192


def pack_frame(meta: FrameMeta, jpeg: bytes, datagram_size=1200):
    if not 256 <= datagram_size <= 1400:
        raise ValueError("datagram_size must be 256..1400")
    metadata = meta.model_dump_json().encode()
    if len(metadata) > MAX_METADATA_BYTES:
        raise ValueError("metadata too large")
    body = struct.pack("!I", len(metadata)) + metadata + jpeg
    if not jpeg or len(body) > MAX_FRAME_BYTES:
        raise ValueError("invalid frame size")
    chunk_size = datagram_size - HEADER.size
    count = (len(body) + chunk_size - 1) // chunk_size
    if count > MAX_CHUNKS:
        raise ValueError("too many fragments")
    crc = zlib.crc32(body)
    for index in range(count):
        yield HEADER.pack(MAGIC, 2, meta.mission_id.encode(), meta.uav_id.encode(),
                          meta.stream_epoch.bytes, meta.frame_id, count, index, len(body), crc) + body[index*chunk_size:(index+1)*chunk_size]


class Reassembler:
    def __init__(self, mission_id, uav_id, timeout_s=2, max_pending=16, max_buffer_bytes=16_000_000):
        self.mission_id, self.uav_id = mission_id, uav_id
        self.timeout_s = timeout_s
        self.max_pending, self.max_buffer_bytes = max_pending, max_buffer_bytes
        self.pending = OrderedDict()
        self.completed = OrderedDict()
        self.bytes_pending = 0
        self.stats = Counter()

    def _remove(self, key):
        entry = self.pending.pop(key)
        self.bytes_pending -= entry["bytes"]

    def cleanup(self, now=None):
        now = time.monotonic() if now is None else now
        for key, entry in list(self.pending.items()):
            if now - entry["created"] >= self.timeout_s:
                self._remove(key)
                self.stats["expired"] += 1
        while self.completed and (len(self.completed) > 512 or now-next(iter(self.completed.values())) > 30):
            self.completed.popitem(last=False)

    def feed(self, packet: bytes, address, now=None):
        now = time.monotonic() if now is None else now
        self.cleanup(now)
        try:
            if not HEADER.size < len(packet) <= 1400:
                raise ValueError("invalid datagram size")
            magic, version, mission, uav, epoch, fid, count, index, length, crc = HEADER.unpack_from(packet)
            mission = mission.rstrip(b"\0").decode("ascii")
            uav = uav.rstrip(b"\0").decode("ascii")
            if magic != MAGIC or version != 2 or (mission, uav) != (self.mission_id, self.uav_id):
                raise ValueError("wrong protocol/mission/UAV")
            if not (0 <= index < count <= MAX_CHUNKS and 4 < length <= MAX_FRAME_BYTES):
                raise ValueError("invalid chunk header")
            payload = packet[HEADER.size:]
            key = (mission, uav, epoch, fid)
            if key in self.completed:
                self.stats["duplicates"] += 1
                return None
            if key not in self.pending:
                while self.pending and (len(self.pending) >= self.max_pending or self.bytes_pending+len(payload) > self.max_buffer_bytes):
                    self._remove(next(iter(self.pending)))
                    self.stats["evicted"] += 1
                self.pending[key] = {"created": now, "address": address, "signature": (count, length, crc),
                                     "parts": {}, "bytes": 0}
            entry = self.pending[key]
            if entry["address"] != address or entry["signature"] != (count, length, crc):
                raise ValueError("conflicting source/header")
            if index in entry["parts"]:
                if entry["parts"][index] != payload:
                    self._remove(key)
                    raise ValueError("conflicting duplicate")
                self.stats["duplicates"] += 1
                return None
            if entry["bytes"]+len(payload) > length or self.bytes_pending+len(payload) > self.max_buffer_bytes:
                self._remove(key)
                raise ValueError("buffer budget exceeded")
            entry["parts"][index] = payload
            entry["bytes"] += len(payload)
            self.bytes_pending += len(payload)
            if len(entry["parts"]) != count:
                return None
            body = b"".join(entry["parts"][i] for i in range(count))
            self._remove(key)
            if len(body) != length or zlib.crc32(body) != crc:
                raise ValueError("checksum/size mismatch")
            meta_len = struct.unpack_from("!I", body)[0]
            if not 0 < meta_len <= MAX_METADATA_BYTES or 4+meta_len >= len(body):
                raise ValueError("invalid metadata envelope")
            meta = FrameMeta.model_validate_json(body[4:4+meta_len])
            if meta.key != (mission, uav, str(UUID(bytes=epoch)), fid):
                raise ValueError("metadata/header identity mismatch")
            self.completed[key] = now
            self.stats["completed"] += 1
            return meta, body[4+meta_len:]
        except (ValueError, UnicodeError, struct.error, KeyError, json.JSONDecodeError):
            self.stats["invalid"] += 1
            return None
