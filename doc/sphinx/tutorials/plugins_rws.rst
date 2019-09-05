.. _plugins_rws:

**************************
GUI Plugins: RobWorkStudio
**************************

The RobWorkStudio GUI is based on Qt, and through the use of Qt plugins,
it is possible to extend RobWorkStudio with user-defined features and behaviour.
Notice that this is very different from the RobWork plugins that makes it possible to extend
the core libraries with new features.
Please see :ref:`plugins_rw` for more information about that subject.

GUI plugins must inherit the rws::RobWorkStudioPlugin class
(see `C++ API <../../apidoc/cpp/doxygen/classrws_1_1RobWorkStudioPlugin.html>`__)
and implement at least some of its functions.
In this section we present two common project templates for creating a GUI plugin:

- A simple template where Qt gui functionality needs to be added in the hpp/cpp plugin files.
- An example where Qt designer is used to create an ui file which describe the graphical layout.  

Common to both templates is how to load the plugin into RobWorkStudio when they have compiled. 
The compiled output will be a dynamically linkable file (in Windows it is a .dll file and in Linux it is .so). 
To use the plugin in your RobWorkStudio installation add the following lines to the 
RobWorkStudio.ini file in the directory where you start RobWorkStudio from::
 
	SamplePlugin\DockArea=2
	SamplePlugin\Filename=libSamplePlugin
	SamplePlugin\Path=PATH_TO_PLUGIN
	SamplePlugin\Visible=true  

Without Qt Designer
===================

This template can be found in the folder RobWorkStudio/example/pluginapp . The files include:

- **SamplePlugin.hpp**: the header file of the plugin
- **SamplePlugin.cpp**: the source file of the plugin
- **SamplePlugin.json**: used by Qt5 for metainformation about the plugin (name, version etc.)
- **resources.qrc**: a Qt resource file that enables compiling images directly into exe/dll/so
- **pa_icon.png**: a sample icon used in the toolbar of RobWorkStudio
- **CMakeLists.txt**: the CMake project file

This example shows how to create a plugin without the use of a ui file from Qt Designer.
The header file for the plugin is shown below. The plugin must inherit the RobWorkStudioPlugin class.
The *open* and *close* functions is called when a WorkCell is opened or closed in RobWorkStudio.
The *initialize* function is called initially when the RobWorkStudio instance is valid.
It can be used to initialize values that depend on the RobWorkStudio instance.

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.hpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.hpp

In the hpp file the Q_PLUGIN_METADATA macro refers to the file SamplePlugin.json.
This file must look like the following:

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.json
   :language: json
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.json

The implementation of the plugin is shown below.
In the constructor two pushbuttons is added to the plugin. Each button is connected to the clickEvent function.
Qt operates with signals and slots, and in this case the clicked() signal from the pushbutton is connect to the clickEvent() slot defined in the plugin.
In the initialize function, a *stateChangedListener* is registered in RobWorkStudio.
This function will be called everytime the state changes.
In the clickEvent() function we can do different computations depending on the which button was pushed. This can be determined based on the Qt sender().

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.cpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.cpp

In the cpp file the constructor refers to the path ":/pa_icon.png".
This is using the Qt resource system. A resources.qrc file is used to define the files that can be used in the plugin.
By using this system, the graphics will be compiled into the plugin binary (the .dll or .so files).
It is up to the user to define a plugin icon.

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/resources.qrc
   :language: xml
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/resources.qrc

The CMake file used for compiling the plugin is shown below:

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/CMakeLists.txt
   :language: cmake
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/CMakeLists.txt

The CMake script will look for RobWork and RobWorkStudio, and it will set the output directories for libraries and executables.
In the last few lines, the CMAKE_AUTOMOC and CMAKE_AUTORCC is enabled, before the library is added and linked to RobWork and RobWorkStudio.
AUTOMOC and AUTORCC are CMake features that takes care of some Qt specific stuff automatically.

Before running CMake, you should set the RW_ROOT and RWS_ROOT environment variables to the path for RobWork and RobWorkStudio respectively.

To compile the plugin you should first create a separate build directory and run CMake from there::

   cmake -DCMAKE_BUILD_TYPE=Release path/to/plugin

On Windows you should specify the generator::

   cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" path/to/plugin

You need to set the build type to the same build type that you used for compiling RobWork. Insert the correct path to the plugin code.

Build the plugin with make (on Linux)::

   make

In Windows, open the SamplePlugin solution in Visual Studio (solution was generated with CMake).

When the .dll or .so file is generated it can be loaded from the RobWorkStudio menu Plugins->Load Plugin.
Alternatively it can be added to RobWorkStudio.ini to be automatically loaded at startup.

With Qt Designer
================

This template can be found in the folder RobWorkStudio/example/pluginUIapp.
It involves the same files as the template `Without Qt Designer`_, but with the additional file SamplePlugin.ui.
The .ui file is a Qt Designer file.
Qt Designer makes it possible to develop the graphical layout of the plugin in a drag and drop fashion.
For more complex plugins this is much more intuitive, than programming it up in C++.

In the following we will discuss the changes compared to the template `Without Qt Designer`_,
so please refer to this section for more details.

The header file for the plugin is shown below.
Compared to the template `Without Qt Designer`_, the header file does not have QPushButton objects defined as private members.
These buttons are instead defined in the .ui file.
By including *ui_SamplePlugin.hpp* and inheriting from *Ui::SamplePlugin*, these buttons can be accessed from our C++ code.

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/SamplePlugin.hpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginUIapp/SamplePlugin.hpp

The implementation of the plugin is shown below.
The constructor is more simple, since the buttons are easily set up with the setupUi(this) call.
All we need to do is to connect the signals and slots.

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/SamplePlugin.cpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginUIapp/SamplePlugin.cpp

The CMake file used for compiling the plugin is shown below:

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/CMakeLists.txt
   :language: cmake
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/CMakeLists.txt

The CMake script is almost identical to the one for the `Without Qt Designer`_ template.
Since we have .ui files, we also enable the CMAKE_AUTOUIC feature.
This way CMake and Qt automatically generates the ui_SamplePlugin.h file that we include in the SamplePlugin.hpp file.

Tips & Examples
===============

Here are some small useful examples that can be used from a plugin

Get CollisionDetector
---------------------------------------

Get the collision detector that is currently used in your RobWorkStudio instance:

.. code-block:: c++

   CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();

Communication between plugins
---------------------------------------

RobWorkStudio has a number of events which can be used by the plugins. A plugin can register for an event, for example by:

.. code-block:: c++

   getRobWorkStudio()->stateChangedEvent().add(boost::bind(&MyPlugin::stateChangedListener, this,_1), this);

which binds the stateChangedListener method of MyPlugin to listen for state changed event.

To see more information about the different event please consult the RobWorkStudio
`API documentation <../../apidoc/cpp/doxygen/classrws_1_1RobWorkStudio.html>`__.

Adding new frames to the workcell from a plugin
-----------------------------------------------

This example describe how one can add his own frames to the workcell through
a user plugin.

Adding frames to the workcell is possible through the StateStructure instance that
is located in a WorkCell. It is important to understand that adding frames to the
state structure will change the static state structure of the workcell (the dynamic state is that
which is located in the State object). Changing the static structure will not directly influence
State objects, that is they are still valid for all frames except the newly added frames.
There exist two methods of making an old state valid for new frames. One is to just assign
the old state with a new one. Though, this will also overwrite any state information that was
saved in the old state, say the configuration of your robot. If you want to preserve the information
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

.. TODO: must be updated!

   Adding drawables from a plugin
   ---------------------------------------

   This example describe how one can add his own drawable to the robwork scene graph, from
   his own RobWorkStudio plugin.
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
          getRobWorkStudio()->getView()->getWorkCellScene()->addDrawable(myFrame, drawableObj);

   Adding collision models from a plugin
   ---------------------------------------

   .. code-block:: c++

      double scale = 1.0; // set a scale, actually not used in RobWork yet
      Transform3D<> transform = makeMyTransform();
      CollisionModelInfo info("myname", transform, scale);

      Accessor::collisionModelInfo().get(*myFrame).push_back(info);

   Getting drawables from a frame
   ---------------------------------------

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
