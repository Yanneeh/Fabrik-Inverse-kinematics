from fabrikSolver import FabrikSolver2D

arm = FabrikSolver2D()

arm.addSegment(100, 90)
arm.addSegment(100, 90)

arm.compute(100, 150)

arm.plot()