# Fabrik inverse kinematics.

A very small and flexible implementation of the Fabrik algorithm. Useful for animation, robotics or other optimization problems.
This module is written in python. 

## Installation

First, download zip or clone repo. Unzip if necessairy and move the *fabrikSolver* file to your project folder.

Second, install dependencies.
```
pip install -r requirements.txt
```

Then import the classes and done.
```python
from fabrikSolver import FabrikSolver2D, FabrikSolver3D
```

## Docs

To see the full documentation. Execute the *docs* file.

```
sh docs.sh
```

## Quickstart

Create a new instance of a fabrik solver class in 2D.

```python
arm = FabrikSolver2D()
```

Add *Segments* to the instance.

```python
arm.addSegment(200 , 70)
arm.addSegment(150 , 90)
arm.addSegment(80 , 20)
```

Move the end effector of the arm to a new x, y coördinate instantly.

```python
arm.compute(300, 100)
```

Plot the *Segments* endpoints after the calculation for debugging and testing.

```python
arm.plot()
```

__Check the *main* file for a runnable example.__

## Usage

The FabrikSolver2D and FabrikSolver3D classes contain functions that affect the movement of segments or visualise the endpoints of segments.

Most of the class methods are the same for 2D and 3D. This is because of the nature of the algorithm. The algorithm could theoretically be used in n dimensions. 

Although it is possible to use the 3D solver in 2D, it is not recommended to mix up 2D and 3D. This will result in some unexpected behaviour if not executed properly. I think it is better to decide whether something should move in 2D or 3D beforehand. This is why the module seperates 2D and 3D completely. 

### __Initialization__
----

#### 2D

Create a new instance of the 2D fabrik solver class. With optional arguments.
The (x, y) coördinates of the function will move the fixed base of the segments chain.
The margin of error will be the distance permitted between the end effector and the target. The smaller this value, the more precise the calculation will be and the more computational recources and/or time the calculation will require.

```python
arm = FabrikSolver2D(x=0, y=0, marginOfError=0.01)
```

The calculation functions only work if there are segments attached. A minimum of two segments is required. The first parameter is the segments length, the second parameter is the inital angle of a segment in degrees.

```python
arm.addSegment(length, angle)
```

#### 3D

Create a new instance of the 3D fabrik solver class. With optinal arguments.
The (x, y, z) coördinates of the function will move the fixed base of the segments chain.
The margin of error will be the distance permitted between the end effector and the target. The smaller this value, the more precise the calculation will be and the more computational recources and/or time the calculation will require.

```python
arm = FabrikSolver3D(x=0, y=0, z=0, marginOfError=0.01)
```

The calculation functions only work if there are segments attached. A minimum of two segments is required. The first parameter is the segments length, the second parameter is the inital angle along the z axis and the third parameter is the inital angle along the y axis.

```python
arm.addSegment(length, zAngle, yAngle)
```

### __Calculation__
----

#### 2D

To move the end effector of the arm to a new point instantly. This calculation is not exact. The distance between the end effector and the arm will be smaller than the margin of error initialized in the contructor.

```python
arm.compute(x, y)
```

To do one iteration of the algorithm to reach a new point. Used in the compute function, but also very useful in simulations or systems with a loop that require converging over time.

```python
arm.iterate(x, y)
```

To check if an endpoint is reachable. Used in the compute function.

```python
if arm.isReachable(x, y):
    print('Endpoint reachable')
```

To check if the distance between the endpoint and the end effector is within the margin of error. Used in the compute function.

```python
if arm.inMarginOfError(x, y):
    print('Reached the endpoint')
```

#### 3D

To move the end effector of the arm to a new point instantly. This calculation is not exact. The distance between the end effector and the arm will be smaller than the margin of error initialized in the contructor.

```python
arm.compute(x, y, z)
```

To do one iteration of the algorithm to reach a new point. Used in the compute function, but also very useful in simulations or systems with a loop that require converging over time.

```python
arm.iterate(x, y, z)
```

To check if an endpoint is reachable. Used in the compute function.

```python
if arm.isReachable(x, y, z):
    print('Endpoint reachable')
```

To check if the distance between the endpoint and the end effector is within the margin of error. Used in the compute function.

```python
if arm.inMarginOfError(x, y, z):
    print('Reached the endpoint')
```

### __Visualization__
----

#### 2D

For debugging and visualizing. This will open a 2D plot with the endpoint of all segments and the base point.
With some optional parameters. Set save to true to save the plot to a png file. Give a name to the png file with the name parameter. The x min, x max, y min and y max are the range of the plot. This works the same as in a graphical calculator.
```python
arm.plot(save=False, name="graph", xMin=-300, xMax=300, yMin=-300, yMax=300)
```

#### 3D

For debugging and visualizing. This will open a 3D plot with the endpoint of all segments and the base point.
With some optional parameters. Set save to true to save the plot to a png file. Give a name to the png file with the name parameter.
```python
arm.plot(save=False, name="graph")
```

## Future Roadmap

* Support for more languages:
    * Javascript
    * Java
    * c++
    * c
* Serial connection to support robotic arms and other systems driven by an embedded system. Arduino, etc.
* Quaternions for rotations because they are awesome.