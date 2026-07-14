import unittest

import numpy as np

from uwb_position import solve_position_3d


class TrilaterationTests(unittest.TestCase):
    def test_exact_ranges_recover_position(self):
        anchors = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.0, 20000.0, 75.0],
                [20000.0, 20000.0, 125.0],
                [20000.0, 0.0, 2000.0],
            ]
        )
        expected = np.array([10000.0, 1000.0, 1000.0])
        ranges = np.linalg.norm(anchors - expected, axis=1)

        actual = solve_position_3d(anchors, ranges, initial=expected)

        np.testing.assert_allclose(actual, expected, atol=1e-3)


if __name__ == "__main__":
    unittest.main()
