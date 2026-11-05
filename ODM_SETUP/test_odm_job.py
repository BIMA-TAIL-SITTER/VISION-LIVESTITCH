import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import odm_job


class JobTests(unittest.TestCase):
    def test_dedup_keeps_originals(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, content in {"a.JPG": b"one", "copy.jpg": b"one", "b.png": b"two", "notes.txt": b"three"}.items():
                (root / name).write_bytes(content)
            photos, duplicates = odm_job.photos_in(root)
            self.assertEqual(len(photos), 2)
            self.assertEqual(duplicates, ["copy.jpg"])
            self.assertEqual(len(list(root.iterdir())), 4)

    def test_submit_records_uuid_and_rejects_repeat(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for index in range(3):
                (root / f"{index}.jpg").write_bytes(bytes([index]))
            node = Mock()
            node.create_task.return_value.uuid = "test-uuid"
            argv = ["odm_job.py", "submit", "--images", str(root), "--name", "mission-UAV1"]
            with patch.object(odm_job, "ROOT", root), patch.object(odm_job, "Node", return_value=node), patch("sys.argv", argv):
                odm_job.main()
                record = json.loads((root / "jobs/mission-UAV1.json").read_text())
                self.assertEqual(record["uuid"], "test-uuid")
                with self.assertRaises(FileExistsError):
                    odm_job.main()
                self.assertEqual(node.create_task.call_count, 1)

    def test_invalid_input_does_not_reserve_job(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(odm_job, "ROOT", root), patch.object(odm_job, "Node"), patch("sys.argv", ["odm_job.py", "submit", "--images", str(root), "--name", "empty"]):
                with self.assertRaises(ValueError):
                    odm_job.main()
                self.assertFalse((root / "jobs").exists())


if __name__ == "__main__":
    unittest.main()
