******************
Inverse Kinematics
******************

Inverse kinematics is the problem of calculating the configuration of a device, :math:`\mathbf{q}`,
such that the transformation from base to TCP frame becomes the desired,
:math:`\mathbf{T}_{base}^{TCP}(\mathbf{q}) = \mathbf{T}_{desired}`.

RobWork provides the InvKinSolver interface for solvers that are able to do this calculation.
The solvers can be grouped into two overall types:

- ClosedFormIK: solvers that can solve the inverse kinematics by analytical expressions.
- IterativeIK: solvers that uses multiple steps to iteratively find a solution. Typically these are using the device Jacobian to do so.

In general the ClosedFormIK solvers can be expected to be computationally efficient,
but these solvers will typically only work for specific robots.
The iterative solvers can be expected to be less efficient, but are also applicable for more generic robots.

Closed Form Solvers
*******************

Universal Robots
==================

C++
------------------

.. literalinclude:: ../../../RobWork/example/cpp/invkin.cpp
   :language: c++
   :linenos:

Python
------------------

.. literalinclude:: ../../../RobWork/example/python/invkin.py
   :language: python
   :linenos:

Java
------------------

.. literalinclude:: ../../../RobWork/example/java/src/rw_java_example/InvKin.java
   :language: java
   :linenos:

LUA
------------------

.. literalinclude:: ../../../RobWork/example/lua/invkin.lua
   :language: lua
   :linenos:

Output
------------------

Output::

 Inverse Kinematics for UR10e.
  Base frame: UR10e.Base
  End/TCP frame: UR10e.Flange
  Target Transform: Transform3D(Vector3D(0.2, -0.2, 0.5), Rotation3D(-1, 0, 1.22465e-16, 0, 1, 0, -1.22465e-16, 0, -1))
 Found 8 solutions.
  Q[6]{-0.12278, 3.02845, 2.16872, -0.484776, -1.5708, -1.69358}
  Q[6]{-0.12278, -1.21998, -2.16872, 1.8179, -1.5708, -1.69358}
  Q[6]{-0.12278, -2.90011, 2.36859, 2.10231, 1.5708, 1.44802}
  Q[6]{-0.12278, -0.705404, -2.36859, -1.63839, 1.5708, 1.44802}
  Q[6]{1.69358, -2.43619, 2.36859, -1.5032, -1.5708, 0.12278}
  Q[6]{1.69358, -0.241482, -2.36859, 1.03928, -1.5708, 0.12278}
  Q[6]{1.69358, -1.92161, 2.16872, 1.3237, 1.5708, -3.01881}
  Q[6]{1.69358, 0.113143, -2.16872, -2.65682, 1.5708, -3.01881}

Kuka IIWA
==================

Iterative Solvers
*******************

Jacobian Solver
==================

Parallel Robots
==================
