.. _interfaces_python:

Python interface
================

Make sure you have both SWIG and python installed. If you are using Ubuntu, make sure to install python3-dev package as well.
You can check if Python interfaces for RobWork are generated when running cmake. There should be lines like: "RobWork: python bindings ENABLED" and
"RobWorkStudio: python bindings ENABLED".

When compiling RobWork, RobWorkStudio and RobWorkSim the python interfaces will be
generated. These consist of two files per project and they are placed in the
libs folder under the relevant build configuration.

Linux example for Debug build configuration::

   RobWork/libs/Debug/_rw.so 
   RobWork/libs/Debug/rw.py 
   RobWorkStudio/libs/Debug/_rws.so 
   RobWorkStudio/libs/Debug/rws.py 
   RobWorkSim/libs/Debug/_rwsim.so 
   RobWorkSim/libs/Debug/rwsim.py 


For the above example loading of the modules in the python interpreter requires the PYTHONPATH environment variable to be set:

.. code-block:: bash

   export PYTHONPATH=/path/to/RobWork/libs/BUILD_CONFIGURATION/:/path/to/RobWorkStudio/libs/BUILD_CONFIGURATION/:/path/to/RobWorkSim/libs/BUILD_CONFIGURATION/:$PYTHONPATH

As an alternative the paths can be set in the python script as well:

.. code-block:: py

   import sys
   sys.path.append('/path/to/RobWork/libs/BUILD_CONFIGURATION')
   sys.path.append('/path/to/RobWorkSim/libs/BUILD_CONFIGURATION')
   sys.path.append('/path/to/RobWorkStudio/libs/BUILD_CONFIGURATION')
   import rw, rws, rwsim
   // now we can use all robwork python bindings

Now all RobWork types, that have bindings, should be available. For some purposes it might
be useful to start an instance of RobWorkStudio:

.. code-block:: py

   rwstudio = rws.getRobWorkStudioInstance()
   // now load a workcell
   rwstudio.postOpenWorkCell('some/workcell.wc.xml')
   // lets get the workcell
   wc = rwstudio.getWorkCell()
   print(wc.getName())

Communicating with plugins
--------------------------

It is often necessary to send messages or data to one or more plugins. For this the
generic event methods on RobWorkStudio is used. These are wrapped in utils such that
the current send methods can be used in python

.. code-block:: py

   rwstudio.send("someStr")
   rwstudio.send("msgId", "some string")
   rwstudio.send("msgId", 0.45235)
   rwstudio.send("msgId", rw.Q(4, 0.1, 0.2, 0.3, 0.5) )
   rwstudio.send("msgId", somePropertyMap )

The first send method use RobWorkStudio::genericEvent the next 4 use RobWorkStudio::genericAnyEvent.
Please take a look in rws/RobWorkStudio.hpp to get an example on using these events in your plugin.

Using path planners in python
-----------------------------

RobWork has several path planners which might be used from python. If we assume that a workcell with a
6 DOF robot named **UR1** has been loaded then a planner can be executed as follows:

.. code-block:: py

   // we need the workcell to get a handle to the robot
   wc = rwstudio.getWorkCell()
   dev = wc.findDevice("UR1")
   state = rwstudio.getState()
   cd = rwstudio.getCollisionDetector()
   planner = rw.QToQPlanner_makeRRT(cd,dev,state)

   // now the planner is ready to be used. We define the configurations
   // in which the robot should start and end
   q_from = rw.Q(6,0,-1,0,0,0,0)
   q_to = rw.Q(6, 3,0.2,1,-1,0,0)
   result = planner.query(q_from,q_to)

We could also chose a query with a timeout or with some other rw::pathplanning::StopCriteria

.. code-block:: py

   result = planner.query(q_from,q_to, 10.0) 
   stopCriteria = rw.StopCriteria_stopCnt(100)
   result = planner.query(q_from,q_to,stopCriteria)