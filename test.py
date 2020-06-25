import unittest
import math
from fabrikSolver import FabrikSolver2D, FabrikSolver3D

class TestFabrikSolver2D(unittest.TestCase):

    # Test for the correct lengths of segments.
    def test_lengths_segments(self):
        
        arm = FabrikSolver2D()

        arm.addSegment(100, 30)
        arm.addSegment(50, 20)

        arm.compute(50, 100)

        distance1 = math.sqrt(math.pow(arm.segments[0].point[0] - arm.basePoint[0], 2) + math.pow(arm.segments[0].point[1] - arm.basePoint[1], 2))
        distance2 = math.sqrt(math.pow(arm.segments[1].point[0] - arm.segments[0].point[0], 2) + math.pow(arm.segments[1].point[1] - arm.segments[0].point[1], 2))

        # Assert almost equal because the sqrt function is not exact.
        self.assertAlmostEqual(distance1, 100)
        self.assertAlmostEqual(distance2, 50)

    # Test for correct end effector point in compute.
    def test_compute(self):
        arm = FabrikSolver2D()

        arm.addSegment(100, 30)
        arm.addSegment(50, 20)

        arm.compute(50, 100)

        # Assert almost equal to account for the margin of error.
        self.assertAlmostEqual(arm.segments[-1].point[0], 50, None, None, arm.marginOfError)
        self.assertAlmostEqual(arm.segments[-1].point[1], 100, None, None, arm.marginOfError)

    # Test for correct endpoint in iterate.
    def test_iterate(self):
        arm = FabrikSolver2D()

        arm.addSegment(100, 30)
        arm.addSegment(50, 20)

        arm.iterate(50, 100)


        self.assertEqual(arm.segments[-1].point[0], 49.99797448038654)
        self.assertEqual(arm.segments[-1].point[1], 100.00270069282436)


class TestFabrikSolver3D(unittest.TestCase):

    # Test for the correct lengths of segments.
    def test_lengths_segments(self):
        
        arm = FabrikSolver3D()

        arm.addSegment(100, 30, 100)
        arm.addSegment(50, 20, 50)

        arm.compute(50, 100, 40)

        distance1 = math.sqrt(math.pow(arm.segments[0].point[0] - arm.basePoint[0], 2) + math.pow(arm.segments[0].point[1] - arm.basePoint[1], 2) + math.pow(arm.segments[0].point[2] - arm.basePoint[2], 2))
        distance2 = math.sqrt(math.pow(arm.segments[1].point[0] - arm.segments[0].point[0], 2) + math.pow(arm.segments[1].point[1] - arm.segments[0].point[1], 2) + math.pow(arm.segments[1].point[2] - arm.segments[0].point[2], 2))

        # Assert almost equal because the sqrt function is not exact.
        self.assertAlmostEqual(distance1, 100)
        self.assertAlmostEqual(distance2, 50)

    # Test for correct end effector point in compute.
    def test_compute(self):
        arm = FabrikSolver3D()

        arm.addSegment(100, 30, 100)
        arm.addSegment(50, 20, 50)

        arm.compute(50, 100, 40)

        # Assert almost equal to account for the margin of error.
        self.assertAlmostEqual(arm.segments[-1].point[0], 50, None, None, arm.marginOfError)
        self.assertAlmostEqual(arm.segments[-1].point[1], 100, None, None, arm.marginOfError)
        self.assertAlmostEqual(arm.segments[-1].point[2], 40, None, None, arm.marginOfError)

    # Test for correct endpoint in iterate.
    def test_iterate(self):
        arm = FabrikSolver3D()

        arm.addSegment(100, 30, 100)
        arm.addSegment(50, 20, 50)

        arm.compute(50, 100, 40)

        self.assertEqual(arm.segments[-1].point[0], 49.99979215649577)
        self.assertEqual(arm.segments[-1].point[1], 100.00122154015793)
        self.assertEqual(arm.segments[-1].point[2], 39.99944571380158)

if __name__ == '__main__':
    unittest.main()