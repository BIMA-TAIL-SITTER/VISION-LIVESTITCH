import random
import struct
import uuid
import zlib
from SENDER2.protocol import HEADER, Reassembler, pack_frame
from SENDER2.runtime import StreamOrder


def test_reorder_duplicate_and_checksum(meta_factory):
    meta = meta_factory()
    data = bytes(range(256))*40
    packets = list(pack_frame(meta,data))
    assert max(map(len,packets)) <= 1200
    random.Random(2).shuffle(packets)
    r = Reassembler("test01","UAV1")
    assert r.feed(packets[0],("a",1),now=0) is None
    assert r.feed(packets[0],("a",1),now=0) is None
    results = [r.feed(p,("a",1),now=0.1) for p in packets[1:]]
    got = next(x for x in results if x)
    assert got[0].key == meta.key and got[1] == data
    assert r.bytes_pending == 0
    assert all(r.feed(p,("a",1),now=0.2) is None for p in packets)


def test_lost_packet_expires_even_without_traffic(meta_factory):
    r = Reassembler("test01","UAV1",timeout_s=1)
    packets = list(pack_frame(meta_factory(),b"x"*6000))
    for p in packets[:-1]:
        assert r.feed(p,("a",1),now=0) is None
    r.cleanup(now=2)
    assert r.bytes_pending == 0 and not r.pending
    assert r.stats["expired"] == 1


def test_same_frame_two_uavs_are_independent(meta_factory):
    one,two = Reassembler("test01","UAV1"),Reassembler("test01","UAV2")
    pa = list(pack_frame(meta_factory(),b"A"*4000))
    pb = list(pack_frame(meta_factory(uav_id="UAV2"),b"B"*4000))
    outa,outb = None,None
    for a,b in zip(pa,pb):
        assert one.feed(b,("b",2)) is None
        outa = one.feed(a,("a",1)) or outa
        outb = two.feed(b,("b",2)) or outb
    assert outa[1] == b"A"*4000 and outb[1] == b"B"*4000


def test_conflicting_source_cannot_mix_chunks(meta_factory):
    r = Reassembler("test01","UAV1")
    packets = list(pack_frame(meta_factory(),b"A"*5000))
    r.feed(packets[0],("source_a",1))
    for p in packets[1:]:
        assert r.feed(p,("source_b",2)) is None
    assert r.stats["invalid"] > 0
    outputs = [r.feed(p,("source_a",1)) for p in packets[1:]]
    assert outputs[-1][1] == b"A"*5000


def test_invalid_index_and_checksum_are_rejected(meta_factory):
    packets = list(pack_frame(meta_factory(),b"A"*4000))
    fields = list(HEADER.unpack_from(packets[0]))
    fields[7] = fields[6]
    r = Reassembler("test01","UAV1")
    assert r.feed(HEADER.pack(*fields)+packets[0][HEADER.size:],("a",1)) is None
    corrupted = packets[-1][:-1]+bytes([packets[-1][-1]^1])
    for p in packets[:-1]+[corrupted]:
        assert r.feed(p,("a",1)) is None
    assert r.stats["invalid"] == 2


def test_memory_budget_and_pending_limit(meta_factory):
    r = Reassembler("test01","UAV1",max_pending=2,max_buffer_bytes=2500)
    for i in range(20):
        packet = next(pack_frame(meta_factory(frame_id=i),b"A"*5000))
        r.feed(packet,("a",1),now=i/100)
        assert len(r.pending)<=2 and r.bytes_pending<=2500
    assert r.stats["evicted"] > 0


def test_restart_epoch_and_late_packets(meta_factory):
    first = meta_factory(frame_id=5)
    order = StreamOrder()
    assert order.admit(first)
    assert not order.admit(meta_factory(frame_id=4))
    assert not order.admit(first)
    restarted = meta_factory(frame_id=0,stream_epoch=uuid.uuid4(),
                             stream_started_unix_ns=first.stream_started_unix_ns+1)
    assert order.admit(restarted)
    assert not order.admit(meta_factory(frame_id=99))
    assert order.admit(restarted.model_copy(update={"frame_id":1}))


def test_different_epochs_can_reassemble_without_collision(meta_factory):
    r = Reassembler("test01","UAV1")
    a = list(pack_frame(meta_factory(),b"a"*5000))
    b = list(pack_frame(meta_factory(stream_epoch=uuid.uuid4()),b"b"*5000))
    got = []
    for x,y in zip(a,b):
        for p in (x,y):
            result = r.feed(p,("a",1))
            if result: got.append(result[1])
    assert got == [b"a"*5000,b"b"*5000]
