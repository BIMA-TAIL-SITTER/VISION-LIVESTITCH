import json
from pathlib import Path
from queue import Full
import socket
import time
import uuid
import cv2
import numpy as np
from fastapi.testclient import TestClient
from SENDER2.api import create_app
from SENDER2.models import GroundConfig, UAVConfig
from SENDER2.protocol import pack_frame
from SENDER2.runtime import GroundRuntime
from SENDER2.worker import worker_main


def unused_ports():
    sockets = [socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for _ in range(2)]
    try:
        for s in sockets: s.bind(("127.0.0.1",0))
        return [s.getsockname()[1] for s in sockets]
    finally:
        for s in sockets: s.close()


def wait_for(predicate,timeout=15):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate(): return
        time.sleep(0.05)
    raise AssertionError("condition did not become true before timeout")


def test_two_uav_processes_udp_restart_loss_api_and_shutdown(workdir,engine_config,meta_factory,textured):
    ports = unused_ports()
    cfg = GroundConfig(mission_id="test01",bind_host="127.0.0.1",output_dir=str(workdir),
        engine=engine_config,allow_simulation=True,queue_size=4,reassembly_timeout_s=0.3,
        uavs=[UAVConfig(uav_id="UAV1",port=ports[0]),UAVConfig(uav_id="UAV2",port=ports[1])])
    runtime = GroundRuntime(cfg)
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    runtime.start()
    root = runtime.root
    def send(meta,image,port,drop=False):
        ok,jpeg = cv2.imencode(".jpg",image)
        assert ok
        packets = list(pack_frame(meta,jpeg.tobytes()))
        for i,p in enumerate(reversed(packets)):
            if drop and i==0: continue
            sock.sendto(p,("127.0.0.1",port))
            # Model a paced radio rather than saturating OS socket buffers.
            time.sleep(0.0002)
    try:
        wait_for(lambda:all(s["worker"].get("state")=="ready" for s in runtime.snapshot()["uavs"].values()))
        assert runtime.streams["UAV1"]["process"].pid != runtime.streams["UAV2"]["process"].pid
        first = meta_factory()
        send(first,textured[:,:320],ports[0])
        # Distinct world content and same frame number: must remain independent.
        send(meta_factory(uav_id="UAV2"),np.flip(textured[:,:320],axis=0).copy(),ports[1])
        wait_for(lambda:all(s["worker"].get("stitched",0)>=1 for s in runtime.snapshot()["uavs"].values()))
        with TestClient(create_app(runtime,manage_lifecycle=False)) as client:
            assert client.get("/health").status_code==200
            a,b = client.get("/uavs/UAV1/preview.jpg"),client.get("/uavs/UAV2/preview.jpg")
            assert a.status_code==b.status_code==200 and a.content != b.content
            assert client.get("/uavs/unknown/status").status_code==404
            assert client.get("/uavs/UAV1/segments/0000/secret.txt").status_code==404
            send(meta_factory(frame_id=1),textured[:,10:330],ports[0],drop=True)
            wait_for(lambda:runtime.snapshot()["uavs"]["UAV1"]["reassembly"].get("expired",0)>=1)
            assert runtime.snapshot()["uavs"]["UAV1"]["worker"]["processed"]==1
            restarted = meta_factory(stream_epoch=uuid.uuid4(),stream_started_unix_ns=first.stream_started_unix_ns+1)
            send(restarted,textured[:,20:340],ports[0])
            wait_for(lambda:runtime.snapshot()["uavs"]["UAV1"]["worker"].get("segment")==1)
            send(meta_factory(frame_id=2),textured[:,30:350],ports[0])
            wait_for(lambda:runtime.snapshot()["uavs"]["UAV1"]["network"].get("late_or_duplicate",0)>=1)
            assert client.get("/uavs/UAV2/status").json()["worker"]["segment"]==0
    finally:
        sock.close()
        runtime.stop()
    assert all(not s["process"].is_alive() for s in runtime.streams.values())
    assert (root/"UAV1/segments/0000/mosaic.png").exists()
    assert (root/"UAV1/segments/0001/mosaic.png").exists()
    assert (root/"UAV2/segments/0000/mosaic.png").exists()
    assert (root/"UAV1/segments/0001/coverage.png").exists()
    shutdown = json.loads((root/"shutdown.json").read_text())
    assert all(s["worker"]["state"]=="stopped" for s in shutdown["uavs"].values())


def test_live_queue_has_real_capacity(workdir,engine_config):
    cfg = GroundConfig(output_dir=str(workdir),engine=engine_config,queue_size=1)
    runtime = GroundRuntime(cfg)
    q = runtime.ctx.Queue(maxsize=cfg.queue_size)
    try:
        q.put_nowait(b"first")
        try:
            q.put_nowait(b"second")
            raise AssertionError("queue should reject excess input")
        except Full:
            pass
        assert q.get(timeout=2)==b"first"
    finally:
        q.close()
        q.join_thread()


def test_flight_mode_refuses_missing_calibration(workdir,engine_config):
    cfg = GroundConfig(output_dir=str(workdir))
    runtime = GroundRuntime(cfg)
    try:
        runtime.start()
        raise AssertionError("flight mode must require measured calibration")
    except ValueError as exc:
        assert "calibration" in str(exc)


def test_worker_drops_expired_queue_items_and_reports_decode_error(workdir,engine_config,meta_factory):
    runtime = GroundRuntime(GroundConfig(output_dir=str(workdir),engine=engine_config))
    incoming = runtime.ctx.Queue(maxsize=4)
    outgoing = runtime.ctx.Queue(maxsize=64)
    stop = runtime.ctx.Event()
    process = runtime.ctx.Process(target=worker_main,args=(engine_config.model_dump(),None,incoming,outgoing,
        stop,str(workdir/"worker"),5))
    # Queue-age rejection must happen before decoder touches the deliberately bad bytes.
    incoming.put((meta_factory().model_dump(mode="json"),b"invalid",time.monotonic_ns()-60_000_000_000))
    process.start()
    try:
        status = workdir/"worker/status.json"
        wait_for(lambda:status.exists() and json.loads(status.read_text()).get("stale_dropped")==1)
        incoming.put((meta_factory(frame_id=1).model_dump(mode="json"),b"invalid",time.monotonic_ns()))
        wait_for(lambda:json.loads(status.read_text()).get("errors")==1)
        error = json.loads(status.read_text())
        assert error["result"]["status"]=="error" and "Image" in error["result"]["reason"]
    finally:
        stop.set()
        process.join(timeout=10)
        if process.is_alive():
            process.terminate()
            process.join(timeout=3)
        incoming.close()
        outgoing.close()
        incoming.cancel_join_thread()
        outgoing.cancel_join_thread()
    assert process.exitcode==0
    final = json.loads(status.read_text())
    assert final["state"]=="stopped" and final["stale_dropped"]==1 and final["errors"]==1
