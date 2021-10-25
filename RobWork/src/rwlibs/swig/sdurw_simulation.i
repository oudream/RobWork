%module sdurw_simulation

%{
#include <rw/sensor/StereoCameraModel.hpp>
#include <rw/sensor/RGBDCameraModel.hpp>
#include <rw/sensor/TactileArrayModel.hpp>
#include <rw/sensor/FTSensorModel.hpp>
#include <rw/sensor/Scanner25DModel.hpp>
#include <rwlibs/simulation/FrameGrabber.hpp>
#include <rwlibs/simulation/FrameGrabber25D.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/graphics/SceneViewer.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/models.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::simulation;
using namespace rwlibs::control;
using namespace rw::graphics;
%}

%include <std_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_control.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_control.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_control.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_control.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%nodefaultctor rwlibs::simulation::SimulatedSensor;
%{
    #include <rwlibs/simulation/SimulatedSensor.hpp>
%}
%include <rwlibs/simulation/SimulatedSensor.hpp>

%template (SimulatedSensorPtr) rw::core::Ptr<rwlibs::simulation::SimulatedSensor>;
%template (SimulatedSensorPtrVector) std::vector<rw::core::Ptr<SimulatedSensor> >;

%nodefaultctor rwlibs::simulation::SimulatedController;
%{
    #include <rwlibs/simulation/SimulatedController.hpp>
%}
%include <rwlibs/simulation/SimulatedController.hpp>

%template (SimulatedControllerPtr) rw::core::Ptr<rwlibs::simulation::SimulatedController>;
%template (SimulatedControllerPtrVector) std::vector<rw::core::Ptr<SimulatedController> >;

%nodefaultctor Simulator;
%{
    #include <rwlibs/simulation/Simulator.hpp>
%}
%include <rwlibs/simulation/Simulator.hpp>

%template (SimulatorPtr) rw::core::Ptr<rwlibs::simulation::Simulator>;

%feature("director") rwlibs::simulation::FrameGrabber;
%{
    #include <rwlibs/simulation/FrameGrabber.hpp>
%}
%include <rwlibs/simulation/FrameGrabber.hpp>
%template (FrameGrabberPtr) rw::core::Ptr<rwlibs::simulation::FrameGrabber>;


%{
    #include <rwlibs/simulation/FrameGrabber25D.hpp>
%}
%include <rwlibs/simulation/FrameGrabber25D.hpp>
%template (FrameGrabber25DPtr) rw::core::Ptr<rwlibs::simulation::FrameGrabber25D>;

%{
    #include <rwlibs/simulation/GLFrameGrabber.hpp>
%}
%include <rwlibs/simulation/GLFrameGrabber.hpp>
NAMED_OWNEDPTR(GLFrameGrabber,rwlibs::simulation::GLFrameGrabber);

%extend rw::core::Ptr<rwlibs::simulation::GLFrameGrabber> {
    rw::core::Ptr<rwlibs::simulation::FrameGrabber> asFrameGrabberPtr() { return $self->cast<rwlibs::simulation::FrameGrabber>(); }
}

%{
    #include <rwlibs/simulation/GLFrameGrabber25D.hpp>
%}
%include <rwlibs/simulation/GLFrameGrabber25D.hpp>
NAMED_OWNEDPTR(GLFrameGrabber25D,rwlibs::simulation::GLFrameGrabber25D);
%extend rw::core::Ptr<GLFrameGrabber25D> {
    rw::core::Ptr<rwlibs::simulation::FrameGrabber25D> asFrameGrabber25DPtr() { return $self->cast<rwlibs::simulation::FrameGrabber25D>(); }
}

%{
    #include <rwlibs/simulation/SimulatedCamera.hpp>
%}
%include <rwlibs/simulation/SimulatedCamera.hpp>
NAMED_OWNEDPTR(SimulatedCamera,rwlibs::simulation::SimulatedCamera);

%{
    #include <rwlibs/simulation/SimulatedScanner2D.hpp>
%}
%include <rwlibs/simulation/SimulatedScanner2D.hpp>
NAMED_OWNEDPTR(SimulatedScanner2D,rwlibs::simulation::SimulatedScanner2D);

%{
    #include <rwlibs/simulation/SimulatedScanner25D.hpp>
%}
%include <rwlibs/simulation/SimulatedScanner25D.hpp>
NAMED_OWNEDPTR(SimulatedScanner25D,rwlibs::simulation::SimulatedScanner25D);

