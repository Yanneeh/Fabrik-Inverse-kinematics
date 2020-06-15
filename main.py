from fabrikSolver import FabrikSolver2D, FabrikSolver3D

arm = FabrikSolver3D()

arm.addSegment(100, 0, 0)
arm.addSegment(100, 0, 0)

arm.compute(100, 150, 50)

arm.plot()