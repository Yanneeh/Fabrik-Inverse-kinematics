# Fabrik inverse kinematics.

## Installation

First, install dependencies.
```
pip install -r requirements.txt
```

Dowload zip, unzip and move module to project folder. Then import and done. __WARNING:__ module name may change in the future. This module is written in python.

```python
from fabrik import Arm
```

## Functions

The Arm class contains some functions that affect movement or visualize a process.

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
