# Fabrik inverse kinematics.

## Functions

The Fabrik class contains some functions that affect movement or visualize a process. This module is written in python.

Define a new instance of the arm class.

```python
arm = Arm()
```

The arm functions only works if there are some segments attached. A minimum of two segments is required. First parameter is the segments length. Second parameter is the angle of a segment.

```python
arm.addSegment( 'length of segment' , 'Angle of segment' )
```

To calculate how the arm should move.

```python
arm.calc2D( 'x', 'y' )
```

For debugging and visualizing. This will open a 2D plot.

```python
arm.plt2D()
```

__Full 3D support is coming soon!__ 
