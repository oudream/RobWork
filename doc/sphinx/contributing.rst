************
Contributing
************

When contributing to the RobWork project as a developer, you need to follow the guidelines provided here.
You should `Setup your IDE`_ to follow the `Coding Guidelines`_. Please read the coding guidelines carefully.
Finally, some guidelines is given on `Committing Code`_ to our main repository.


Setup your IDE
==============

If you use one of the following IDEs, please consider importing one of the provided style configurations:

- :download:`Eclipse IDE <../rw_codestyle_eclipse.xml>`
- :download:`QtCreator IDE <../rw_codestyle_qtcreator.xml>`
- :download:`CLion IDE <../rw_codestyle_clion.xml>`
- :download:`Visual Studio IDE <../rw_codestyle_visualstudio.vssettings>`

In case you are developing as a part of SDU Robotics, please configure your editor to add the RobWork license to your .cpp and .hpp files::

  /******************************************************************************
   * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
   * Faculty of Engineering, University of Southern Denmark
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   *     http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   ******************************************************************************/

If you develop under a different license, your code might not be suitable for addition to the main RobWork repository before additional review.

Coding Guidelines
=================

This section will present the coding guidelines that are used in RobWork.
We base the coding policy on the following guide lines:

http://geosoft.no/development/cppstyle.html

With the following exceptions:

- We name the C++ header files .hpp (instead of .h)
- We name the C++ source files .cpp (instead of .c++)
- We prefix member variables with _ (instead of using _ as a suffix)
- We use an indentation level of 4 characters
- We include RobWork headers before anything else

In-depth explanations follow.

Naming of source files:
***********************

Source files are named in UpperCamelCase (Java style) the following suffixes should be used:

- C++ header files: .hpp
- C++ source files: .cpp

As a rule of thumb: There should be one .hpp and one .cpp file for each class. The .hpp and .cpp
file should have the same name as the class

Include guards
**************

Use the following includeguards:

.. code-block:: cpp

   #ifndef RW_PACKAGENAME_CLASSNAME_HPP
   #define RW_PACKAGENAME_CLASSNAME_HPP
   ... // your code goes here
   #endif // RW_PACKAGENAME_CLASSNAME_HPP

example:

.. code-block:: cpp

   #ifndef RW_MODELS_SERIALDEVICE_HPP
   #define RW_MODELS_SERIALDEVICE_HPP
   ...
   #endif // RW_MODELS_SERIALDEVICE_HPP

Use of namespaces
*****************

.. code-block:: cpp

   namespace rw {
       namespace packagename {

       }
   }

Avoid using "using namespace" in .hpp files. This violates the principle of namespaces.
"using namespace" is only allowed in .cpp files.

Class definitions
*****************

Place public members before protected members, place protected members before private members

.. code-block:: cpp

   class SerialDevice
   {
       public:
           void someFunction();

       protected:
           ...

       private:
           ...
   };

Documentation
*************

We use doxygen for documentations, doxygen tags should start with a "\@" (JavaDoc style). Brief member
descriptions should be prefixed by \@brief

We use the same writing style as used in the Java API (see http://java.sun.com/j2se/1.5.0/docs/api/)

Example of good brief member descriptions:

- \@brief Removes all of the elements of this collection
- \@brief Constructs an ActionEvent object
- \@brief Returns a parameter string identifying this action event

Example of bad brief member descriptions:

- This method is used for finding the square root

There should be a space between class members and the next documentation block

Right:

.. code-block:: cpp

   class Test {
       public:
           // @brief Performs the first test
           void test1();

           // @brief Performs the second test
           void test2();
   };

Wrong:

.. code-block:: cpp

   class Test {
       public:
           // @brief Performs the first test
           void test1();
           // @brief Performs the second test
           void test2();
   };

Indentation
***********

We use indentation level 4. Please be careful to setup your IDE to use spaces and not tabs.

Notation for math
*****************

When possible use the following notation for code and documentation:

============================ =============================== ================================= ===== =================================
Documentation                Doxygen                         Sphinx                            Code  Description
============================ =============================== ================================= ===== =================================
:math:`{{}^{a}{\bf{T}}_{b}}` \\f$\\abx{a}{b}{\\bf{T}}\\f$    \:math\:\`{{}^{a}{\\bf{T}}_{b}}\` aTb   Transform a to b (or b wrt. a)
**x**                        \\f$\\b{x}\\f$                  \*\*x\*\*                         X     Pose
**d**                        \\bf{d}                         \*\*d\*\*                         d     Vector
:math:`\hat{\bf{k}}\theta`   \\f$\\hat{\\bf{k}}\\theta\\f$   \:math\:\`\\hat{\\bf{k}}\\theta\` k     EAA, equivalent angle and axis
:math:`\bf{\nu}`             \\bf{\\nu}                      \:math\:\`\\bf{\\nu}\`            V     VelocityScrew
**v**                        \\bf{v}                         \*\*v\*\*                         v     Linear velocity
:math:`\bf{\omega}`          \\bf{\\omega}                   \:math\:\`\\bf{\\omega}\`         w     Angular velocity
**q**                        \\f$\\bf{q}\\f$                 \*\*q\*\*                         q     Joint configuration
============================ =============================== ================================= ===== =================================

Include files
*************

.hpp files should be included in the follwing order:

- (for .cpp files only) ClassName.hpp
- .hpp files from same namespace
- RobWork .hpp files
- ext includes
- other includes
- boost includes
- stl includes

Example.: (SerialDevice.cpp)

.. code-block:: cpp

   #include "SerialDevice.hpp"

   #include "DependentJoint.hpp"
   #include "Joint.hpp"

   #include <rw/kinematics/Frame.hpp>
   #include <rw/kinematics/Kinematics.hpp>

   #include <vector>

Feel free to add spaces to indicate the include groups as shown above. Sort the files in each group lexicographically.

For source files in test, example and demo use the above rules but include the
RobWork files as library files instead of local files (for instance use <rw/models/Joint.hpp> instead of "Joint.hpp")

Try to reduce .hpp dependencies
*******************************

Try to reduce .hpp dependencies by not including more .hpp files than absolutely necessary.
Use forward declarations when possible.

Use tests
*********

Do not remove or comment-out tests from the test directory. When you add new classes or functions, be sure to create a test of it.
New tests should be written based on the Google Test framework, while older ones are written as Boost tests.

Use the RobWork smart pointer
*****************************

All classes which are expected to be passed as pointers should declare a pointer typedef using the
RobWork smart pointer rw::common::Ptr.

.. code-block:: cpp

   class MyClass;

   // A pointer to a MyClass
   typedef rw::common::Ptr<MyClass> MyClassPtr;

Classes taking pointers to objects should likewise use the smart pointer to determine ownership
and avoid memory leaks.

.. note::
   We are currently considering to directly use the std::smart_ptr available in C++11 instead of the RobWork smart pointer.

Templates
===============

To combine all of the best practices described here, an example of a .hpp and .cpp file is provided.
These can also be used at templates when developing new classes.

.hpp file
*********

.. code-block:: cpp

   /********************************************************************************
    * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
    * Faculty of Engineering, University of Southern Denmark 
    * 
    * Licensed under the Apache License, Version 2.0 (the "License");
    * you may not use this file except in compliance with the License.
    * You may obtain a copy of the License at
    *
    *     http://www.apache.org/licenses/LICENSE-2.0
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    ********************************************************************************/

   #ifndef RW_MODELS_SERIALDEVICE_HPP
   #define RW_MODELS_SERIALDEVICE_HPP

   /**
    * @file SerialDevice.hpp
    */

   #include "JointDevice.hpp"

   #include <vector>

   namespace rw {
       namespace models {
           /** @addtogroup models */
           //! @{

           /**
            * @brief The device for a serial chain.
            *
            * SerialChain is like JointDevice except that SerialChain has the
            * additional guarantee that the joints lie on a single parent to child
            * path of the kinematic tree.
            */
           class SerialDevice : public JointDevice
           {
               public:
                   //! @brief smart pointer type to this class
                   typedef rw::common::Ptr<SerialDevice> Ptr;
                   //! @brief smart pointer type to this const class
                   typedef rw::common::Ptr< const SerialDevice > CPtr;

                   /**
                    * @brief Constructor
                    *
                    * @param first [in] the base frame of the robot
                    * @param last [in] the end-effector of the robot
                    * @param name [in] name of device
                    * @param state [in] the connectedness of the frames
                    */
                   SerialDevice(
                           kinematics::Frame* first,
                           kinematics::Frame* last,
                           const std::string& name,
                           const kinematics::State& state);

                   //! @brief destructor
                   virtual ~SerialDevice() {}

                   /**
                    * @brief Frames of the device.
                    *
                    * This method is being used when displaying the kinematic
                    * structure of devices in RobWorkStudio. The method really
                    * isn't of much use for everyday programming.
                    *
                    * @return list of raw Frame pointers.
                    */
                   const std::vector<kinematics::Frame*>& frames() const;

                   ...

               private:
                   std::vector<kinematics::Frame*> _kinematicChain;
           };
           //! @}
       } // end models namespace
   } // end rw namespace

   #endif // end include guard

.cpp file
*********

.. code-block:: cpp

   /********************************************************************************
    * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
    * Faculty of Engineering, University of Southern Denmark 
    * 
    * Licensed under the Apache License, Version 2.0 (the "License");
    * you may not use this file except in compliance with the License.
    * You may obtain a copy of the License at
    *
    *     http://www.apache.org/licenses/LICENSE-2.0
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    ********************************************************************************/

   #include "SerialDevice.hpp"

   #include "DependentJoint.hpp"
   #include "Joint.hpp"

   #include <rw/kinematics/Frame.hpp>
   #include <rw/kinematics/Kinematics.hpp>

   #include <vector>

   using namespace rw::common;
   using namespace rw::kinematics;
   using namespace rw::math;
   using namespace rw::models;

   namespace
   {
       std::vector<Joint*> getJointsFromFrames(const std::vector<Frame*>& frames)
       {
           std::vector<Joint*> active;
           ...
           return active;
       }
   }

   SerialDevice::SerialDevice(Frame* first,
                              Frame* last,
                              const std::string& name,
                              const State& state):
       JointDevice(name, first,
               last,getJointsFromFrames(...),state),
       _kinematicChain(getChain(first, last, state))
   {
   }

   const std::vector<Frame*>& SerialDevice::frames() const
   {
       return _kinematicChain;
   }

Committing Code
===============

In order to contribute with code and fixes to the main RobWork project, you have to first make a personal fork of the project.
In this fork you can do your own work.
After pushing your work to you personal project, it is then possible to make a GitLab merge request in order to merge the changes into the main project.
A merge request will require some checks to succeed, and will need approval by maintainers at SDU Robotics, before being merged.

The full process is as follows:

#. Fork the repository.
   Go to https://gitlab.com/sdurobotics/RobWork and click Fork in the upper right corner.
   Follow the guide and place it somewhere in your own namespace.
#. Clone your new forked project to your local machine, using either SSH or HTTP:

   - **git clone git@gitlab.com:<username>/RobWork.git**
   - **git clone https://gitlab.com/<username>/RobWork.git**

#. Go to your cloned project folder and add the upstream project as an extra remote, using either SSH or HTTPS:

   - **git remote add upstream git@gitlab.com:sdurobotics/RobWork.git**
   - **git remote add upstream https://gitlab.com/sdurobotics/RobWork.git**

#. To update your project, checkout your own master, fetch changes from upstream, and merge them into your own master.
   Optionally, you can push your new updated master to your own fork on GitLab.com:

   - **git checkout master**
   - **git fetch upstream**
   - **git merge upstream/master**
   - **git push origin master**

#. Before adding your work, checkout a new branch with a meaningful name for your changes:

   - **git checkout -b <branch_name>**

#. Use **git status** to check if there is uncommitted work on your branch. Use **git add** to stage changes for commit. You can check what is staged with **git status** and **git diff --cached**.
#. Test that you can still compile the RobWork projects locally. Run the tests to make sure that you did not break any tests.
#. If everything seems to work, commit your staged changes with **git commit -m "Some meaningful description of your changes"**.
   If you have not set your username and email, you will be guided on how to do so.
#. After you have made your commits, push the commit to your project at GitLab with **git push origin <branch_name>**.
#. Go to your project page on GitLab.com and click on "Merge Requests" and "New merge request".
#. Select the source branch from your own project and the target branch sdurobotics/RobWork master branch.
#. Click "Compare branches and continue". Put in a description and finalize your merge request.
#. When the merge request is submitted, a pipeline is launched. This pipeline tests that the most important parts of RobWork compile (a minimal compilation).
#. If the pipeline fails, fix the error, commit the change, and push it to your branch. The new change will cause the pipeline to run again.
#. When the pipeline succeeds, you will have to wait for a maintainer from SDU Robotics to accept the request.
   When the request is accepted and the pipeline succeeds, you will be able to finish the merge (this step might also be done directly by a maintainer).
#. After the merge, a much larger pipeline is triggered. This checks that RobWork compiles and that tests still work. This is done on all platforms supported by RobWork.
#. If this last pipeline fails, you should fix the issue as quickly as possible. You will need to create a new merge request with a fix.

When you commit:

- Always specify a commit message stating what was changed (this makes it possible for other people to see why you made changes and, more importantly, why you changed it)
