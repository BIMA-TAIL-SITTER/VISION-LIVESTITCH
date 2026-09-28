import os
import shutil
import sys
import unittest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from service import StitchingSession, StitchConfig

class TestStitchingSessionClaimStitch(unittest.TestCase):
    def setUp(self):
        self.test_root = os.path.join(os.path.dirname(__file__), "_tmp_session_test")
        os.chdir(os.path.dirname(os.path.dirname(__file__)))  # repo root, service.py uses relative "./sessions"
        config = StitchConfig(sessionId="test_claim_session")
        self.session = StitchingSession("test_claim_session", config)

    def tearDown(self):
        shutil.rmtree("./sessions/test_claim_session", ignore_errors=True)

    def test_claim_stitch_succeeds_when_not_stitching(self):
        self.assertTrue(self.session.claim_stitch())
        self.assertTrue(self.session.is_stitching)

    def test_claim_stitch_fails_when_already_stitching(self):
        self.session.claim_stitch()
        self.assertFalse(self.session.claim_stitch())

    def test_finish_stitch_allows_reclaiming(self):
        self.session.claim_stitch()
        self.session.finish_stitch()
        self.assertTrue(self.session.claim_stitch())

    def test_claim_stitch_records_accepted_count_at_claim_time(self):
        self.session.accepted_images = ["a.jpg", "b.jpg", "c.jpg"]
        self.session.claim_stitch()
        self.assertEqual(self.session.last_stitch_count, 3)

    def test_filters_are_independent_per_session_instance(self):
        config2 = StitchConfig(sessionId="test_claim_session_2")
        session2 = StitchingSession("test_claim_session_2", config2)
        try:
            self.session.gps_filter.should_accept(-7.25, 112.77)
            # session2's filter must be untouched by session1's call
            accepted, dist = session2.gps_filter.should_accept(10.0, 20.0)
            self.assertTrue(accepted)
            self.assertEqual(dist, 0.0)
        finally:
            shutil.rmtree("./sessions/test_claim_session_2", ignore_errors=True)


if __name__ == '__main__':
    unittest.main()