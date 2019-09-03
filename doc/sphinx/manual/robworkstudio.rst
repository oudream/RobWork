*******************
RobWorkStudio
*******************

The main goal of RobWorkStudio is to implement functionality for vizualising
a RobWork workcell and to implement a plugin infrastructure that enables easy
installation of user functionality.
 
*RobWorkStudio* is simple and convinient front-end for visualizing
RobWork Workcells and their operation.

Default Plugins
===============

Plugins in RobWorkStudio define the functionallity wether it be native plugins
or user defined plugins.

.. |jog| image:: ../../../RobWorkStudio/src/rwslibs/jog/jog.png
   :height: 20
   :width: 20
.. |log| image:: ../../../RobWorkStudio/src/rwslibs/log/log.png
   :height: 20
   :width: 20
.. |lua| image:: ../../../RobWorkStudio/src/rwslibs/lua/lua.png
   :height: 20
   :width: 20
.. |planning| image:: ../../../RobWorkStudio/src/rwslibs/planning/planning.png
   :height: 20
   :width: 20
.. |playback| image:: ../../../RobWorkStudio/src/rwslibs/playback/playback.png
   :height: 20
   :width: 20
.. |propertyview| image:: ../../../RobWorkStudio/src/rwslibs/propertyview/propertyview.png
   :height: 20
   :width: 20
.. |sensors| image:: ../../../RobWorkStudio/src/rwslibs/sensors/sensors.png
   :height: 20
   :width: 20
.. |treeview| image:: ../../../RobWorkStudio/src/rwslibs/treeview/treeview.png
   :height: 20
   :width: 20
.. |wceditor| image:: ../../../RobWorkStudio/src/rwslibs/workcelleditorplugin/wceditoricon.png
   :height: 20
   :width: 20

|jog| Jog: Provides functionality for jogging around the robots in a workcell.

|log| Log: Displays the default log in RobWorkStudio

|lua| LUA & LUA Editor: Provides a simple editor for writing and executing lua scripts.

|planning| Planning: Enables the user call motion planners and plan paths.

|playback| Playback: This plugin enables recording and playback of TimedStatePaths.

|propertyview| PropertyView: The propertyview can be used to display and edit properties associated to frames in the workcell.

|sensors| Sensors: This plugin can display output from simulated camera and range scanners in the workcell.

|treeview| Treeview: Shows the frame structure of the workcell.

|wceditor| WorkCell editor: Edit the XML definition of the WorkCell.
The editor provides help to the user and makes it easier to set up a new WorkCell.

Extra Plugins
=============

.. |gtask| image:: ../../../RobWorkStudio/src/images/pa_icon.png
   :height: 20
   :width: 20
.. |atask| image:: ../../../RobWorkStudio/src/rwslibs/atask/atask_icon.png
   :height: 20
   :width: 20

|gtask| Grasp Task

|atask| Assembly Task

User Plugins
============

To create your own plugin, copy one of the example plugins which can be found within the example
directory under RobWorkStudio. User plugins can currently only be developed using C++.
The pluginUIapp provides an example in which QT designer (GUI editor)
is used to design the user interface. The pluginapp provides a simple example without the dependency
of a GUI building tool.

To compile the plugin you need to perform the following steps

- Edit the CMakeLists.txt file to ensure that the variables "RW_ROOT" and "RWSTUDIO_ROOT" points to you RobWork and RobWorkStudio directories.
- call "cmake ." to generate build files
- call "make" to build the plugin.

Once the plugin is build you need to tell RobWorkStudio to load it. This is done by editing the RobWorkStudio.ini file. If the RobWorkStudio.ini file does not exist you can copy the RobWorkStudio.ini.template from the bin directory. Within the template file you may have to remove the existing plugins and add the following::

	MyPlugin\DockArea=1
	MyPlugin\Filename=libmyplugin
	MyPlugin\Path=../../MyPlugin/
	MyPlugin\Visible=false

Be sure that the MyPlugin\\Path points to where your library has been generated and that
MyPlugin\\Filename is correct. You should not add any file extension (this is resolved automatically).

When you start RobWorkStudio it will load your plugin.



Tips
----

Here are some small useful examples that can be used from a plugin

Get the collision detector that is currently used in your robwork studio
instance

.. code-block:: c++

   CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();

Communicating between plugins
*****************************
RobWorkStudio has a number of event which can be used by the plugins. A plugin can register for an event, for example by

.. code-block:: c++

   getRobWorkStudio()->stateChangedEvent().add(boost::bind(&MyPlugin::stateChangedListener, this,_1), this);

which binds the stateChangedListener method of MyPlugin to listen for state changed event.

To see more information about the different event please consult the RobWorkStudio api-doc.


RobWorkStudio specific frame properties
---------------------------------------

Through generic properties in the XML file format, RobWork allows
for adding user specific information to frames. In this section RobWorkStudio specific
properties will be listed. Meaning properties that only makes sense for RobWorkStudio and
not RobWork.

Camera property
***************

A property describing a camera pinhole model can be added to a frame. The camera view can
then be visualized in RobWorkStudio. The property string looks like this::

   <Field of view Y> <width> <height>

example:

.. code-block:: xml

   <Property name="Camera">60 640 480</Property>

You can currently only change views between cameras using Ctrl + the key [1-9], were 1 is the default
3rd person view.

**Important!**

- Multiple cameras are supported but only one camera property per frame!
- The width and height has no real dimension its the proportion between them that matters
- The camera looks in the negative Z-axis direction of the frame
- Field of view is in degree and is defined in the Y-axis

Useful examples
----------------

Adding new frames to the workcell from a plugin
***********************************************

This example describe how one can add his own frames to the workcell through
a user plugin.

Adding frames to the workcell is possible through the StateStructure instance that
is located in a WorkCell. It is important to understand that adding frames to the
state structure will change the static state structure of the workcell (the dynamic state is that
which is located in the State object). Changing the static structure will not directly influence
State objects, that is they are still valid for all frames except the newly added frames.
There exist two methods of making an old state valid for new frames. One is to just assign
the old state with a new one. Though, this will also overwrite any state information that was
saved in the old state, say the configuration of tour robot. If you want to preserve the information
in the old state and still make it valid for newly added frames you would need to upgrade it. You
can upgrade a state **oldstate** using either StateStructure instance **stateStruct** or another
state **newstate**. Following is an example of how:

.. code-block:: c++

   // using another state to upgrade
   oldstate.upgradeTo(newstate); // oldstate is upgraded to the structure of the newstate
   // using state structure to upgrade
   oldstate = stateStruct.upgrade( oldstate );

Following is an example of how to add a new frame to the workcell from your own plugin

.. code-block:: c++

   State oldState; // this is your old state
   Frame *newFrame = make_new_frame(); // create your frame
   getRobWorkStudio()->getWorkcell()->getStructure()->addFrame(newFrame,parentFrame);
   // now update the oldState with the new state
   oldState = getRobWorkStudio()->getWorkCell()->getStructure()->upgradeState(oldState);
   // now this is VERY important, remember to update the RobWorkStudio state
   getRobWorkStudio()->setState(oldState);

Adding drawables from a plugin
******************************

This example describe how one can add his own drawable to the robwork scene graph, from
his own robworkstudio plugin.
First we need to create the drawable, next we need to find the frame we want to
connect it too, and lastly add it to the WorkCellGLDrawer of RWStudio. The following
code snippet show the creation of a user specified render which is used to construct
a drawable. One could also use the DrawableFactory to create a drawable from either
file or a primitive string (Cube,Box,Cylinder etc.). Next a frame called "myFrame" is
searched for in the workcell. If the frame is found then a mapping from myFrame to
the user drawable is created in the WorkCellGLDrawer (SceneGraph).

.. code-block:: c++

   MyRender *renderObj = new MyRender( .. );
   Drawable *drawableObj = new Drawable(boost::shared_ptr<Render>(renderObj));
   Frame *myFrame = getRobWorkStudio()->getWorkCell()->findFrame("myFrame");
   if(drawableFrame != NULL)
       getRobWorkStudio()->getWorkCellGLDrawer()->addDrawableToFrame(myFrame, drawableObj);

Adding collision models from a plugin
*************************************

.. code-block:: c++

   double scale = 1.0; // set a scale, actually not used in RobWork yet
   Transform3D<> transform = makeMyTransform();
   CollisionModelInfo info("myname", transform, scale);

   Accessor::collisionModelInfo().get(*myFrame).push_back(info);

Getting drawables from a frame
******************************

This code snippet will copy all drawables associated with the frame **frameWithDrawables**
into the vector **drawables**.

.. code-block:: c++

   std::vector<Drawable*> drawables;
   Frame *frameWithDrawables; // specify the frame where your drawables are placed
   getWorkCellGLDrawer()->getAllDrawables(state, frameWithDrawables, drawables);

The next code snippet will copy all drawables associated to any frame in the workcell
into the vector **drawables**.

.. code-block:: c++

   std::vector<Drawable*> drawables;
   getWorkCellGLDrawer()->getAllDrawables(state, getWorkCell(), drawables);
