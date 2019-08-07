***************
Motion Planning
***************

Motion planning is about planning a collision free path from one robot configuration to another, avoiding any obstacles in the workspace. In the core RobWork library (rw::pathplanning namespace), there is an interface for such planners called QToQPlanner. RobWork provides a pathplanning library with implementation of various planning algorithms. These are found under the namespace rwlibs::pathplanners and implements the QToQPlanner interface:

+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+
|ARWPlanner  | Adaptive Random Walk Planner                                | `API <http://www.robwork.org/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1ARWPlanner.html>`__ |
+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+
|PRMPlanner  | Probabilistic RoadMap Planner                               | `API <http://www.robwork.org/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1PRMPlanner.html>`__ |
+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+
|RRTPlanner  | Rapidly-exploring Random Tree Planner                       | `API <http://www.robwork.org/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1RRTPlanner.html>`__ |
+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+
|SBLPlanner  | Single-query Bi-directional Lazy collision checking Planner | `API <http://www.robwork.org/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1SBLPlanner.html>`__ |
+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+
|Z3Planner   | Z3 Method                                                   | `API <http://www.robwork.org/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1Z3Planner.html>`__  |
+------------+-------------------------------------------------------------+---------------------------------------------------------------------------------------------------+

See the API documentation for each of these classes for references to litterature and more information about possible variants of the algorithms.

Example Scene
=============

We will use the SimplePA10Demo scene for demonstration of pathplanning.
This scene has a Mitsubishi PA-10 robot mounted on a gantry.
To move from one location in the scene to another requires pathplanning to avoid collisions.

.. figure:: ../graphics/scene_collection/SinglePA10Demo.png

    SinglePA10Demo scene from the :ref:`scene_collection` (RobWorkData).

RobWorkStudio Planning Plugin
=============================

Open the Planning |planning|, Jog |jog| and Log |log| plugins.

.. |planning| image:: ../graphics/icons/planning.png
   :height: 20
   :width: 20
.. |jog| image:: ../graphics/icons/jog.png
   :height: 20
   :width: 20
.. |log| image:: ../graphics/icons/log.png
   :height: 20
   :width: 20

C++
===========

.. code-block:: c++

   #include <rw/kinematics/State.hpp>
   #include <rw/loaders/WorkCellLoader.hpp>
   #include <rw/math/Q.hpp>
   #include <rw/models/Device.hpp>
   #include <rw/models/CompositeDevice.hpp>
   #include <rw/pathplanning/PlannerConstraint.hpp>
   #include <rw/pathplanning/QToQPlanner.hpp>
   #include <rw/proximity/CollisionStrategy.hpp>
   #include <rw/proximity/CollisionDetector.hpp>
   #include <rw/proximity/ProximityData.hpp>
   #include <rw/trajectory/Path.hpp>
   #include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
   #include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

   using rw::common::ownedPtr;
   using rw::kinematics::State;
   using rw::loaders::WorkCellLoader;
   using rw::math::Q;
   using namespace rw::models;
   using namespace rw::proximity;
   using namespace rw::pathplanning;
   using rw::trajectory::QPath;
   using rwlibs::pathplanners::RRTPlanner;
   using rwlibs::proximitystrategies::ProximityStrategyFactory;

   #define WC_FILE "RobWorkData/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

   int main(int argc, char** argv) {
       const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
       if (wc.isNull())
           RW_THROW("WorkCell could not be loaded.");
       const Device::Ptr gantry = wc->findDevice("Gantry");
       const Device::Ptr pa10 = wc->findDevice("PA10");
       if (gantry.isNull())
           RW_THROW("Gantry device could not be found.");
       if (pa10.isNull())
           RW_THROW("PA10 device could not be found.");

       const State defState = wc->getDefaultState();
       const Device::Ptr device = ownedPtr(new CompositeDevice(gantry->getBase(),
               wc->getDevices(), pa10->getEnd(), "Composite", defState));

       const CollisionStrategy::Ptr cdstrategy =
               ProximityStrategyFactory::makeCollisionStrategy("PQP");
       if (cdstrategy.isNull())
           RW_THROW("PQP Collision Strategy could not be found.");
       const CollisionDetector::Ptr collisionDetector =
               ownedPtr(new CollisionDetector(wc, cdstrategy));
       const PlannerConstraint con =
               PlannerConstraint::make(collisionDetector, device, defState);
       const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(con, device);

       const Q beg(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0);
       const Q end(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0);

       ProximityData pdata;
       State state = defState;
       device->setQ(beg, state);
       if (collisionDetector->inCollision(state, pdata))
           RW_THROW("Initial configuration in collision! can not plan a path.");
       device->setQ(end, state);
       if (collisionDetector->inCollision(state, pdata))
           RW_THROW("Final configuration in collision! can not plan a path.");

       QPath result;
       if (planner->query(beg, end, result)) {
           std::cout << "Planned path with " << result.size();
           std::cout << " configurations" << std::endl;
       }

       return 0;
   }

Python
==============

.. code-block:: python

   import sys
   sys.path.append('/path/to/RobWork/libs/relwithdebinfo/')
   from rw import *
   from rw_pathplanners import RRTPlanner
   from rw_proximitystrategies import ProximityStrategyFactory

   WC_FILE = "/path/to/RobWorkData/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

   if __name__ == '__main__':
       wc = WorkCellLoaderFactory.load(WC_FILE)
       if wc.isNull():
           raise Exception("WorkCell could not be loaded")
       gantry = wc.findDevice("Gantry")
       pa10 = wc.findDevice("PA10")
       if gantry.isNull():
           raise Exception("Gantry device could not be found.")
       if pa10.isNull():
           raise Exception("PA10 device could not be found.")

       defState = wc.getDefaultState()
       device = ownedPtr(CompositeDevice(gantry.getBase(), wc.getDevices(),
                                pa10.getEnd(), "Composite", defState))

       cdstrategy = ProximityStrategyFactory.makeCollisionStrategy("PQP")
       if cdstrategy.isNull():
           raise Exception("PQP Collision Strategy could not be found.")
       collisionDetector = ownedPtr(CollisionDetector(wc, cdstrategy))
       con = PlannerConstraint.make(collisionDetector, device.asDeviceCPtr(), defState)
       planner = RRTPlanner.makeQToQPlanner(con, device.asDevicePtr())

       beg = Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0)
       end = Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0)

       pdata = ProximityData()
       state = defState
       device.setQ(beg, state)
       if collisionDetector.inCollision(state, pdata):
           raise Exception("Initial configuration in collision!")
       device.setQ(end, state)
       if collisionDetector.inCollision(state, pdata):
           raise Exception("Final configuration in collision!")

       result = PathQ()
       if planner.query(beg, end, result):
           print("Planned path with " + str(result.size()) + " configurations")

Java
============

.. code-block:: java

   import java.lang.String;
   import org.robwork.LoaderRW;
   import org.robwork.rw.*;
   import static org.robwork.rw.rw.ownedPtr;
   import org.robwork.rw_pathplanners.RRTPlanner;
   import org.robwork.rw_proximitystrategies.ProximityStrategyFactory;

   public class Main {
       public static final String WC_FILE =
               "/path/to/RobWorkData/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml";

       public static void main(String[] args) throws Exception {
           LoaderRW.load("rw");
           LoaderRW.load("rw_pathplanners");
           LoaderRW.load("rw_proximitystrategies");

           WorkCellPtr wc = WorkCellLoaderFactory.load(WC_FILE);
           if (wc.isNull())
               throw new Exception("WorkCell could not be loaded.");
           DevicePtr gantry = wc.findDevice("Gantry");
           DevicePtr pa10 = wc.findDevice("PA10");
           if (gantry.isNull())
               throw new Exception("Gantry device could not be found.");
           if (pa10.isNull())
               throw new Exception("PA10 device could not be found.");

           State state = wc.getDefaultState();
           CompositeDevicePtr device = ownedPtr(new CompositeDevice(gantry.getBase(),
                   wc.getDevices(), pa10.getEnd(), "Composite", defState));

           CollisionStrategyPtr cdstrategy =
                   ProximityStrategyFactory.makeCollisionStrategy("PQP");
           if (cdstrategy.isNull())
               throw new Exception("PQP Collision Strategy could not be found.");
           CollisionDetectorPtr collisionDetector = ownedPtr(
                   new CollisionDetector(wc, cdstrategy));
           PlannerConstraint con = PlannerConstraint.make(collisionDetector,
                   device.asDeviceCPtr(), defState);
           QToQPlannerPtr planner = RRTPlanner.makeQToQPlanner(con, device.asDevicePtr());

           final Q beg = new Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0);
           final Q end = new Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0);

           ProximityData pdata = new ProximityData();
           device.setQ(beg, state);
           if (cd.inCollision(state, pdata))
               throw new Exception("Initial configuration in collision!");
           device.setQ(end, state);
           if (cd.inCollision(state, pdata))
               throw new Exception("Final configuration in collision!");

           PathQ result = new PathQ();
           if (planner.query(beg, end, result)) {
               System.out.print("Planned path with " + result.size());
               System.out.println(" configurations");
           }
       }
   }

.. _lua:

LUA
===========

This example shows how pathplanning can be done in a LUA script.
You can run this script directly in the RobWorkStudio LUA plugin.
If you want to run the script without RobWorkStudio, see the :ref:`lua_standalone` section.

.. code-block:: lua
   :linenos:

   local WC_FILE = "/path/RobWorkData/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

   local wc = WorkCellLoaderFactory.load(WC_FILE)
   if wc:isNull() then
       error("WorkCell could not be loaded")
   end
   local gantry = wc:findDevice("Gantry")
   local pa10 = wc:findDevice("PA10")
   if gantry:isNull() then
       error("Gantry device could not be found.")
   end
   if pa10:isNull() then
       error("PA10 device could not be found.")
   end

   local state = wc:getDefaultState()
   local device = ownedPtr(CompositeDevice(gantry:getBase(), wc:getDevices(),
                           pa10:getEnd(), "Composite", state))

   local cdstrategy = ProximityStrategyFactory.makeCollisionStrategy("PQP")
   if cdstrategy:isNull() then
       error("PQP Collision Strategy could not be found.")
   end
   local collisionDetector = ownedPtr(CollisionDetector(wc, cdstrategy))
   local con = PlannerConstraint.make(collisionDetector, device:asDeviceCPtr(), state)
   local planner = RRTPlanner.makeQToQPlanner(con, device:asDevicePtr())

   local beg = Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0)
   local fin = Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0)

   local pdata = ProximityData()
   device:setQ(beg, state)
   if collisionDetector:inCollision(state, pdata) then
       error("Initial configuration in collision!")
   end
   device:setQ(fin, state)
   if collisionDetector:inCollision(state, pdata) then
       error("Final configuration in collision!")
   end

   local result = PathQ()
   if planner:query(beg, fin, result) then
       print("Planned path with " .. result:size() .. " configurations")
   end

In lines 1-14 the WorkCell is loaded, and the two devices 'Gantry' and 'PA10' are extracted.
Rember to check is the returned smart pointers are null, as this would indicate that something went wrong.
If you continue without checking, you will likely end up with segmentation errors.

In lines 16 and 17, we first get the default state of the workcell. This is used to construct a new CompositeDevice.
The CompositeDevice will make the two devices act as one device to the pathplanning algorithms.
As the PA10 device is placed at the end of the Gantry device, we specify the base frame to be the PA10 base frame, and the end frame to be the PA10 end frame.
The name of the device will be "Composite" and we construct the CompositeDevice from all the devices in the WorkCell (assuming there is only the two).
If you only have one device to plan for, there is obviously no need for constructing a CompositeDevice. Instead, use the device in the WorkCell directly.

In lines 20-23 a CollisionStrategy is created. We base this on the PQP strategy. Remember to check that the returned smart pointer is not null before continuing.

In lines 24-26 a CollisionDetector is created based on this collision strategy. The detector is then wrapped in a PlannerConstraint.
Finally, a RRTPlanner is constructed based on the PlannerConstraint. The planner works for our CompositeDevice.

In lines 28-39 the initial and goal configurations are defined. Before planning, we first check that these are collision free.
The CollisionDetector uses the ProximityData structure to speed up collision detection by performing caching inbetween calls to inCollision.

Finally, in line 31-44, we do the actual pathplanning query. The result will be stored in the PathQ object, and we print the size of the path.

LUA API References
*********************
There is currently no separate API documentation for the LUA interface.
Instead, see the references for Python and Java in :ref:`api_ref`. These are very similar to the LUA interface.

.. _lua_standalone:

Standalone LUA
********************

The code above will work when executed from the RobWorkStudio Lua plugin.
It is also possible to execute the LUA script in a standalone script without using RobWorkStudio.
In the RobWork/bin directory there will be a executable called 'lua' which is the LUA interpreter that can be used to execute standalone scripts.
The script to execute must be given as the first argument to this program.

.. code-block:: lua
   :linenos:

   package.cpath = package.cpath .. ";/path/to/RobWork/libs/relwithdebinfo/lib?_lua.so"
   require("rw")
   require("rw_pathplanners")
   require("rw_proximitystrategies")

   function openpackage (ns)
     for n,v in pairs(ns) do
       if _G[n] ~= nil then
         print("name clash: " .. n .. " is already defined")
       else
         _G[n] = v
       end
     end
   end

   openpackage(rw)
   openpackage(rw_pathplanners)
   openpackage(rw_proximitystrategies)

   -- then all of the above

You must tell LUA where to find the native RobWork libraries.
This is done in the first line by appending to the LUA cpath.
In line 2-4 the necessary native libraries are loaded.
The question mark in line 1 is automatically substituted by the names given to require.

Lines 6 to 18 is optional.
By default, the imported package functions are refered to by scoped names, such as 'rw_pathplanners.RRTPlanner'.
By using the openpackage function, all these scoped names are moved to the global table. That means you can refer directly to the RobWork types, for instance with 'RRTPlanner' instead of the longer 'rw_pathplanners.RRTPlanner'.
To just import a single type to global namespace, you could instead specify

.. code-block:: lua

   local RRTPlanner = rw_pathplanners.RRTPlanner

After this initial import of the native libraries, the script in the :ref:`lua` section can be run.

.. _api_ref:

API References
==============

Below is a list of relevant API references to the types used in the example, in the order they are used.
Use this if you want to know more about options and finetuning of the algorithms.

+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| C++ API                                                                                                                                                                 | Python API Reference                                        | Java API Reference                                                       | Javadoc                                                                                                                                                |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::loaders::WorkCellLoader::Factory <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1loaders_1_1WorkCellLoader_1_1Factory.html>`__                                 | :py:class:`rw.WorkCellLoaderFactory`                        | :java:type:`org.robwork.rw.WorkCellLoaderFactory`                        | `org.robwork.rw.WorkCellLoaderFactory <../graphics/javadoc/org/robwork/rw/WorkCellLoaderFactory.html>`__                                               |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::models::Device::Ptr <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1models_1_1Device.html>`__                                                                  | :py:class:`rw.DevicePtr`                                    | :java:type:`org.robwork.rw.DevicePtr`                                    | `org.robwork.rw.DevicePtr <../graphics/javadoc/org/robwork/rw/DevicePtr.html>`__                                                                       |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::kinematics::State <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1kinematics_1_1State.html>`__                                                                 | :py:class:`rw.State`                                        | :java:type:`org.robwork.rw.State`                                        | `org.robwork.rw.State <../graphics/javadoc/org/robwork/rw/State.html>`__                                                                               |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::models::CompositeDevice <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1models_1_1CompositeDevice.html>`__                                                     | :py:class:`rw.CompositeDevice`                              | :java:type:`org.robwork.rw.CompositeDevice`                              | `org.robwork.rw.CompositeDevice <../graphics/javadoc/org/robwork/rw/CompositeDevice.html>`__                                                           |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
|                                                                                                                                                                         | :py:class:`rw.CompositeDevicePtr`                           | :java:type:`org.robwork.rw.CompositeDevicePtr`                           | `org.robwork.rw.CompositeDevicePtr <../graphics/javadoc/org/robwork/rw/CompositeDevicePtr.html>`__                                                     |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rwlibs::proximitystrategies::ProximityStrategyFactory <http://www.robwork.dk/apidoc/nightly/rw/classrwlibs_1_1proximitystrategies_1_1ProximityStrategyFactory.html>`__ | :py:class:`rw_proximitystrategies.ProximityStrategyFactory` | :java:type:`org.robwork.rw_proximitystrategies.ProximityStrategyFactory` | `org.robwork.rw_proximitystrategies.ProximityStrategyFactory <../graphics/javadoc/org/robwork/rw_proximitystrategies/ProximityStrategyFactory.html>`__ |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::proximity::CollisionDetector <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1proximity_1_1CollisionDetector.html>`__                                           | :py:class:`rw.CollisionDetector`                            | :java:type:`org.robwork.rw.CollisionDetector`                            | `org.robwork.rw.CollisionDetector <../graphics/javadoc/org/robwork/rw/CollisionDetector.html>`__                                                       |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::pathplanning::PlannerConstraint <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1pathplanning_1_1PlannerConstraint.html>`__                                     | :py:class:`rw.PlannerConstraint`                            | :java:type:`org.robwork.rw.PlannerConstraint`                            | `org.robwork.rw.PlannerConstraint <../graphics/javadoc/org/robwork/rw/PlannerConstraint.html>`__                                                       |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rwlibs::pathplanners::RRTPlanner <http://www.robwork.dk/apidoc/nightly/rw/classrwlibs_1_1pathplanners_1_1RRTPlanner.html>`__                                           | :py:class:`rw_pathplanners.RRTPlanner`                      | :java:type:`org.robwork.rw_pathplanners.RRTPlanner`                      | `org.robwork.rw_pathplanners.RRTPlanner <../graphics/javadoc/org/robwork/rw_pathplanners/RRTPlanner.html>`__                                           |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::math::Q <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1math_1_1Q.html>`__                                                                                     | :py:class:`rw.Q`                                            | :java:type:`org.robwork.rw.Q`                                            | `org.robwork.rw.Q <../graphics/javadoc/org/robwork/rw/Q.html>`__                                                                                       |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::proximity::ProximityData <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1proximity_1_1ProximityData.html>`__                                                   | :py:class:`rw.ProximityData`                                | :java:type:`org.robwork.rw.ProximityData`                                | `org.robwork.rw.ProximityData <../graphics/javadoc/org/robwork/rw/ProximityData.html>`__                                                               |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+
| `rw::trajectory::QPath <http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1trajectory_1_1Path.html>`__                                                                  | :py:class:`rw.PathQ`                                        | :java:type:`org.robwork.rw.PathQ`                                        | `org.robwork.rw.PathQ <../graphics/javadoc/org/robwork/rw/PathQ.html>`__                                                                               |
+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------+-------------------------------------------------------------+--------------------------------------------------------------------------+--------------------------------------------------------------------------------------------------------------------------------------------------------+

