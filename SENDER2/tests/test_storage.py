import pytest
from SENDER2 import storage


def test_atomic_write_retries_temporary_destination_lock(workdir, monkeypatch):
    destination = workdir / "status.json"
    destination.write_bytes(b"old complete content")
    real_replace = storage.os.replace
    attempts = []

    def locked_then_available(source, target):
        attempts.append(1)
        assert destination.read_bytes() == b"old complete content"
        if len(attempts) < 3:
            raise PermissionError("temporary sharing violation")
        real_replace(source, target)

    monkeypatch.setattr(storage.os, "replace", locked_then_available)
    monkeypatch.setattr(storage.time, "sleep", lambda _: None)
    storage.atomic_write(destination, b"new complete content")
    assert len(attempts) == 3
    assert destination.read_bytes() == b"new complete content"
    assert not list(workdir.glob("*.part"))


def test_atomic_write_permanent_permission_failure_is_visible(workdir, monkeypatch):
    destination = workdir / "status.json"
    destination.write_bytes(b"old")
    attempts = []

    def always_locked(*args):
        attempts.append(1)
        raise PermissionError("permanent access denial")

    monkeypatch.setattr(storage.os, "replace", always_locked)
    monkeypatch.setattr(storage.time, "sleep", lambda _: None)
    with pytest.raises(PermissionError):
        storage.atomic_write(destination, b"new")
    assert len(attempts) == 8
    assert destination.read_bytes() == b"old"
    assert not list(workdir.glob("*.part"))
