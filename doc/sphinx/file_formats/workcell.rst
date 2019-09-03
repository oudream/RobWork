.. _workcell_format:

***************
WorkCell Format
***************

.. contents:: :local:

Introduction
============

The workcell XML file format have suffix .xml and follow the rules of standard XML.
:ref:`workcells_loading` can be done with with rw::loaders::WorkCellLoader::Factory::load().

The workcell file describes workcell elements such as frames, joints, DAFs, devices and the
properties that relate to these elements such as name, parent name, joint limits,
position and orientation,
collision geometry, drawing geometry and even user defined properties.

The basic structure of a workcell is frames. Frames are connected in a tree like fashion
such that each frame has a parent and [0;many] children. The root of the frame tree is
called the world frame and this frame is the only frame that has no parent. The world frame
is named **WORLD**.

Frames come in many different types: fixed frame, movable frame, prismatic joint, revolute
joint, and so on. They can be freely defined and connected in the workcell file, though often
frames will be grouped into devices.

A device is a robot with one or more joints. It defines a scope where frames that belong to that
device can be described. A device always has a base frame which is the frame belonging to the
device that has a parent that does not belong to the device. That is, a device defines a sub tree
in the frame tree. There exists several device types: serial device, tree device, parallel device
and so on, that allow different sort of connections
between the joints of the device.

User properties can be attached to any frame. A property has a name and a string value. The
property is saved in the frame such that the user can retrieve it and parse the string himself.

A collision setup is a set of include or exclude rules that tell which frame pairs
that should not or should be tested for collision. Ex. two neighboring links on a
robot will usually collide in their attachment points, which is undesirable, the
collision setup should in this case be used to exclude those frames from collision
checking.

Scopes
======

An important thing to notice is that some xml elements are scope dependent. In
general the **refframe** and **refjoint** attributes are optional and if not
defined then scope rules will define them instead.

Ex. when defining a frame in workcell or a device scope the frame will automatically be
attached to the previously defined frame.

.. code-block:: xml

   <WorkCell name="testwc">
       ...
       <Frame name="A" refframe="WORLD" />`

       <Frame name="B" />

       <Frame name="C" refframe="A" />
   </WorkCell>

in the above example A attaches to world and B and C attaches to A.

Another example illustrates how properties defined inside
a frame can omit the refframe attribute.

.. code-block:: xml

   <WorkCell name="testwc">
       ...
       <Frame name="A" refframe="WORLD">
           <Property name="A_Property"> some string value</Property>
       </Frame>

       <Property name="A_nother_Property" refframe="A">some string value</Property>
       ...
   </WorkCell>

RW XML Format
=============

Before going deep into the grammar of the rw xml format some common structure need be
explained. The complete kinematic description is based on a construct named Frame.
The Frame has a name and a transform relative to its parent frame or refframe.
This means that a kinematic
Frame tree can be described with multiple frames. The Frame can be of different
types the simplest being Fixed, which means that the frame transform is unchangeable.

Another common structure is the Property. The property is linket to a frame and has a
name, description and a string value. Properties are used to link different
information to frames. F.ex. collision models and drawables are described using properties.
properties are also used to allow users to attach user specific information to frames.

In the xml file format Frames are grouped logicly in container type elements. These elements
are WorkCell and device types. To avoid name clashes frames belonging to a container type will
have the container name prepended. Example: a frame named "base" specified in a device named
"PA10" in a workcell named "scene" will have the unique name "scene.PA10.base".

The WorkCell element is the root element in the fileformat. It implicitly defines a Fixed frame
named World. This world frame is the root frame in the kinematic frame tree.

::

   workcell
     \*(frame
      | Drawable
      | SerialDevice
      | TreeDevice
      | ParallelDevice
      | MobileDevice
      | CollisionSetup )

WorkCell
--------

**Element** WorkCell

**Attributes**

* **name:** a string identifying the workcell.

**Child elements:**
The DeviceType is any of the device types that can be defined,
see section `Device`_.

::

   rule := *( DeviceType | Frame | Joint | DHJoint | CollisionModel | Drawable | CollisionSetup )

**Example**

.. code-block:: xml

   <WorkCell name="scene">
       ...
   </WorkCell>

Device
------

The different device types are much alike when considering the child elements
that they allow. Though they vary somehow in the implicit rules of frame
attachment.

In general a device defines a scope. This scope has the same name as the
device. Any frames defined inside the scope of a device gets the device
name appended. Ex. given a device "dev" and a frame "base" in the device
the complete frame name becomes: "dev.base"

SerialDevice
^^^^^^^^^^^^

The serial device only allows joints to be connected in a serial chain. And
it also only allows one single endeffector.

**Element** SerialDevice

**Attributes**

* **name:** a string identifying the device.

**Child elements:**

* **Frame**
* **Joint**
* **DHJoint**
* **CollisionModel**
* **Drawable**
* **PosLimit**
* **VelLimit**
* **AccLimit**
* **Property**
* **SerialChain**
* **CollisionSetup**
* **Q**

**Example**

.. code-block:: xml

   <SerialDevice name="RobotArm">
       ...
   </SerialDevice>

TreeDevice
^^^^^^^^^^

The tree device allows joints to be connected in a tree like structure. And
it also allows for multiple endeffectors.

**Element** TreeDevice

**Attributes**

* **name:** a string identifying the device.

**Child elements:**

* **Frame**
* **Joint**
* **DHJoint**
* **CollisionModel**
* **Drawable**
* **PosLimit**
* **VelLimit**
* **AccLimit**
* **Property**
* **SerialChain**
* **CollisionSetup**
* **Q**

**Example**

.. code-block:: xml

   <TreeDevice name="RobotHand">
       ...
   </TreeDevice>

ParallelDevice
^^^^^^^^^^^^^^

The parallel device is like a number of serial devices (with same base) with all endeffectors
rigidly connected together. The initial configuration of the robot is required
to make all endeffectors align in the same pose. For devices that are connected in multiple places,
it is also possible to define so-called junctions. Each Junction must specify two or more chains,
where each chain referes to a list of previously defined SerialChains. Notice that each of these
chains must start and end in equivalent frames. If no junctions are defined, one implicit junction
is created, assuming that each of the defined serial chains must end in the same endeffector frame.

**Attributes**

* **name:** a string identifying the device.

**Child elements:**

* **Frame**
* **Joint**
* **DHJoint**
* **CollisionModel**
* **Drawable**
* **PosLimit**
* **VelLimit**
* **AccLimit**
* **Property**
* **SerialChain**
* **CollisionSetup**
* **Junction**
* **Q**

**Example**

Take the following kinematic structure as an example::

                                   ___
             /--------------------|-C |
            /             ___     |   |
       /-- B ------------|-D-|----|-E |
      /     \    ___     |   |    |___|
     /       ---|-F |    |   |
    /           |   |    |   |
   A -----------|-G-|----|-H-|
                |___|    |___|

Each of the 8 serial chains, A to H, can contain one or more joints.
The boxes show three places where the device must be connected. This can be specified in the device with Junction tags:

.. code-block:: xml

   <ParallelDevice name="RobotHand">
       ...
       <Junction>
           <Chains>C</Chains>
           <Chains>D E</Chains>
       </Junction>
       <Junction>
           <Chains>B D</Chains>
           <Chains>G H</Chains>
       </Junction>
       <Junction>
           <Chains>B F</Chains>
           <Chains>G</Chains>
       </Junction>
        ...
    </ParallelDevice>

Notice that the first serial chain (A) was left out in all the cases, as it is equal for all chains.
For the junction ending after chains C and E, both A and B was left out, as they do not provide any extra information.

MobileDevice
^^^^^^^^^^^^

The mobile device defines a two wheeled mobile robot where the two
wheels are on the same axel displaced from the center of the axel with
some width **AxelWidth**.

**Attributes**

* **name:** a string identifying the device.
* **basename:** name of the mobile device base.

**Child elements:**

* **AxelWidth**
* **LeftWheel**
* **RightWheel**
* **Frame**
* **Joint**
* **DHJoint**
* **CollisionModel**
* **Drawable**
* **PosLimit**
* **VelLimit**
* **AccLimit**
* **Property**
* **SerialChain**
* **CollisionSetup**
* **Q**

**Example**

.. code-block:: xml

   <MobileDevice name="Pioneer" basename="Base">
       ...
   </MobileDevice>

Frame
-----

**Attributes**

* **name:** a string identifying the frame.
* **refframe:** name of the parent frame (optional).
* **type:** a frame type identifier (optional).
* **daf:** boolean defining if the frame is a daf or not (optional)

**Child elements**

!((**RPY** >> **Pos**) | **Transform**) >> \*(**Property** | **CollisionModel** | **Drawable**)

**Example**

.. code-block:: xml

   <Frame name="myframe" refframe="WORLD">
   </Frame>

Joint
^^^^^

**Attributes**


* **name:** a string identifying the frame.
* **refframe:** name of the parent frame (optional).
* **type:** a joint type identifier (Prismatic|Revolute|Universal|Spherical|PrismaticUniversal|PrismaticSpherical).
* **state:** joint state (optional)

**Child elements**

!((**RPY** >> **Pos**) | **Transform**) >> \*(**PosLimit** | **VelLimit** | **AccLimit** | **Depend** | **Property** | **CollisionModel** | **Drawable**)

**Example**

DHJoint
^^^^^^^

A joint that is defined from the Denavit Hartenberg notation.
The Craig DH variant is used. This can only specify Revolute
or Prismatic joints

**Attributes**

* **name:** a string identifying the frame.
* **alpha:**
* **a:**
* **d:**
* **theta:**
* **offset:**
* **state:** joint state (optional)

::

   rule := name >> alpha >> a >> (d >> offset)|(theta >> offset);

**Child elements**

!((**RPY** >> **Pos**) | **Transform**) >> \*(**PosLimit** | **VelLimit** | **AccLimit** | **Depend** | **Property** | **CollisionModel** | **Drawable**)

**Example**

Drawable
--------

**Attributes**


* **name:** the name of the drawable
* **refframe:** the frame that the drawable is to be attached to.
* **colmodel:** (Enabled|Disabled) if enabled the drawable will also
  be used as collision model.

**Child elements**

!((**RPY** >> **Pos**) | **Transform**) >> \*(RGB) >> \*(Polytope | Sphere | Box | Cone | Cylinder)

**Example**

CollisionModel
--------------

**Attributes**


* **name:**
* **refframe:**

**Child elements**

!((**RPY** >> **Pos**) | **Transform**) >> \*(Polytope | Sphere | Box | Cone | Cylinder)

**Example**

Property
--------

**Attributes**


* **name:** name of the property
* **reframe:** frame to attach property to (optional)
* **desc:** description of the property (optional)

**Child elements**

* string value

**Example**
A user property for defining a camera on a frame. The string value
can be parsed by the user to get image dimensions [640;480] and field
of view 40

.. code-block:: xml

   <Property name="Camera" refframe="WORLD" desc="[fovy,width,height]">
       40 640 480
   </Property>

When the 'Camera' property name is used, this will also, when opened in RobWorkStudio, draw the outline of the camera frame.
In this case the camera parameters will always be interpreted in the order fovy,width,height, as shown above, ignoring the description given.

**Example**

A user property for enabling the display of the frame axis. This will automatically execute the TreeView plugin action of turning on the visibility
of the frame axis.

.. code-block:: xml

   <Frame name="my_frame">
       <RPY>0 0 0</RPY>
       <Pos>0 0 1</RPY>
       <Property name="ShowFrameAxis">true</Property>
   </Frame>

Transform
---------

**Attributes**

**Child elements**
All real values are parsed into a rotation matrix **R** and a position **P**

::

   R00 >> R01 >> R02 >> P0 >>
   R10 >> R11 >> R12 >> P1 >>
   R01 >> R21 >> R22 >> P2

**Example**
Loads the identity rotation and the (0.1,0.1,0.1) position. Values are
seperated by whitespace.

.. code-block:: xml

   <Transform>
       1 0 0 0.1
       0 1 0 0.1
       0 0 1 0.1
   </Transform>

RPY
---

**Attributes**

**Child elements**
Loads RPY values seperated by whitespace

::

   roll >> pitch >> yaw

**Example**
A rotation matrix with 90 degree rotation around z-axis

.. code-block:: xml

   <RPY> 90 0 0 </RPY>

RGB
---

**Attributes**

**Child elements**
Defines the simple material colors for the visual model. This material is used if the model does not support the color/material information (e.g. when geometric primitive or STL model is used).

::

   r >> g >> b >> *a

**Example**

Simple material color with RGB values [1.0, 0.0, 0.0] (red color).

.. code-block:: xml

   <RGB> 1.0 0.0 0.0 </RGB>

**Example**

Simple material color with transparency: RGBA values [1.0, 1.0, 0.0 0.5] (transparent yellow color).

.. code-block:: xml

   <RGB> 1.0 1.0 0.0 0.5 </RGB>

Pos
---

**Attributes**

**Child elements**
Loads pos values seperated by whitespace

::

   x >> y >> z

**Example**

.. code-block:: xml

   <Pos> 0.1 0.1 0.2 </Pos>

Geometry
--------

Polytope
^^^^^^^^

**Attributes**


* **file:** the geometry file

**Example**

.. code-block:: xml

   <Polytope file="c:/geometry/object.stl" />

Sphere
^^^^^^

**Attributes**


* **radius:** radius of the sphere in m.
* **level**: (optional, default=20) mesh resolution.

**Example**

.. code-block:: xml

   <Sphere radius="0.05" />

.. code-block:: xml

   <Sphere radius="0.05" level="20" />

Box
^^^

**Attributes**

* **x:** length in x-axis
* **y:** length in y-axis
* **z:** length in z-axis

**Example**

.. code-block:: xml

   <Box x="0.1" y="0.1" z="0.1" />

Cone
^^^^

**Attributes**


* **radius:** radius of bottom circle of cone.
* **z:** height of cone.
* **level:** (optional, default=20) mesh resolution.

**Example**

.. code-block:: xml

   <Cone radius="0.1" z="0.1" />

.. code-block:: xml

   <Cone radius="0.1" z="0.1" level="10"/>

Cylinder
^^^^^^^^

**Attributes**

* **radius:** radius of the cylinder.
* **z:** length of cylinder
* **level:** (optional, default=20) mesh resolution.

**Example**

.. code-block:: xml

   <Cylinder radius="0.1" z="0.1" />

.. code-block:: xml

   <Cylinder radius="0.1" z="0.1" level="20"/>

Tube
^^^^

**Attributes**


* **radius:** radius of outer surface of the tube.
* **thickness:** thickness of the tube surface.
* **z:** height of the tube.
* **level:** (optional, default=20) mesh resolution.

**Example**

.. code-block:: xml

   <Tube radius="0.1" thickness="0.01" z="0.1"/>

.. code-block:: xml

   <Tube radius="0.1" thickness="0.01" z="0.1" level="10"/>

CollisionSetup
--------------

**Attributes**


* **file:** the file where the collision setup is described

**Example**

.. code-block:: xml

   <CollisionSetup file="../mydevice/colsetup.xml" />

PosLimit
--------

**Attributes**


* **refjoint:** the joint which the limit is valid for. (optional)
* **min:** the minimum joint value
* **max:** the maximum joint value

**Example**

.. code-block:: xml

   <PosLimit refjoint="joint1" min="-90" max="90" />

VelLimit
--------

**Attributes**


* **refjoint:** the joint which the limit is valid for. (optional)
* **max:** the maximum joint velocity value

**Example**

.. code-block:: xml

   <VelLimit refjoint="joint1" max="180" />

AccLimit
--------

**Attributes**


* **refjoint:** the joint which the limit is valid for. (optional)
* **max:** the maximum joint acceleration

**Example**

.. code-block:: xml

   <AccLimit refjoint="joint1" max="180" />