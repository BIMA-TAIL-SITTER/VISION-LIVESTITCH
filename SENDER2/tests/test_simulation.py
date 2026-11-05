from SENDER2.tools.simulate_dataset import discover


def test_simulation_deduplicates_bytes_and_prefers_original_name(workdir):
    (workdir / "00001 - Copy.jpg").write_bytes(b"same bytes")
    (workdir / "00001.jpg").write_bytes(b"same bytes")
    (workdir / "00000.jpg").write_bytes(b"different bytes")
    (workdir / "notes.txt").write_text("not an image")
    paths, duplicates, count = discover(workdir)
    assert count == 3
    assert [p.name for p in paths] == ["00000.jpg", "00001.jpg"]
    assert duplicates == [{"skipped": "00001 - Copy.jpg", "same_as": "00001.jpg"}]
    assert len(list(workdir.glob("*.jpg"))) == 3


def test_simulation_can_preserve_duplicate_inputs(workdir):
    for name in ["a.jpg", "b.jpg"]:
        (workdir / name).write_bytes(b"same bytes")
    paths, duplicates, count = discover(workdir, keep_duplicates=True)
    assert len(paths) == count == 2
    assert duplicates == []
