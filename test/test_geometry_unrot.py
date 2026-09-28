import os
import sys
import unittest
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from src.geometry import computeUnRotMatrix

class TestComputeUnRotMatrixDegreesContract(unittest.TestCase):
    def test_zero_pose_returns_identity(self):
        pose = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        matrix = computeUnRotMatrix(pose)
        np.testing.assert_allclose(matrix, np.eye(3), atol=1e-9)

    def test_15_degree_pitch_produces_real_correction_not_57x_too_small(self):
        """
        This is the exact regression the double-conversion bug
        (docs/BUG_REPORT_ATTITUDE_UNIT_MISMATCH.md) would have broken:
        if pose[4] had already been converted to radians before reaching
        here, computeUnRotMatrix's own *pi/180 would shrink the real
        rotation by ~57x, and this matrix would come out nearly identity.
        """
        pose_zero = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        pose_tilted = np.array([0, 0, 0, 0, 15, 0], dtype=np.float64)  # 15 deg pitch

        matrix_zero = computeUnRotMatrix(pose_zero)
        matrix_tilted = computeUnRotMatrix(pose_tilted)

        # A real 15-degree correction must differ substantially from identity --
        # if the input were mistakenly pre-converted to radians (0.2618 passed
        # as "degrees"), the resulting matrix would be nearly indistinguishable
        # from matrix_zero. Assert the difference is NOT tiny.
        diff = np.abs(matrix_tilted - matrix_zero).max()
        self.assertGreater(diff, 0.05, "Correction is too small -- looks like the ~57x bug")


if __name__ == '__main__':
    unittest.main()