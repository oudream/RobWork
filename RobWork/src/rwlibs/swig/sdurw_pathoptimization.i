%module sdurw_pathoptimization

%{
#include <rw/core/Ptr.hpp>
#include <rw/models.hpp>
#include <rw/trajectory.hpp>
#include <rw/core/os.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/math/MetricFactory.hpp>


#include <rwlibs/pathoptimization/clearance/ClearanceCalculator.hpp>
#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

using rw::math::Metric;
using rw::trajectory::Path;

#ifdef RW_WIN32
namespace rwlibs { namespace pathoptimization {
    const std::string ClearanceOptimizer::PROP_STEPSIZE  = "StepSize";
    const std::string ClearanceOptimizer::PROP_LOOPCOUNT = "LoopCount";
    const std::string ClearanceOptimizer::PROP_MAXTIME   = "MaxTime";

    const std::string PathLengthOptimizer::PROP_LOOPCOUNT    = "LoopCount";
    const std::string PathLengthOptimizer::PROP_MAXTIME      = "MaxTime";
    const std::string PathLengthOptimizer::PROP_SUBDIVLENGTH = "SubDivideLength";
}}    // namespace rwlibs::pathoptimization
#endif 
%}

%include <rwlibs/swig/swig_macros.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_proximity.i>
%import <rwlibs/swig/sdurw_pathplanning.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_pathplanning.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_pathplanning.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_pathplanning.*;
%}

%include <rwlibs/swig/typemaps/toClearanceCalculator.i>

%include <rwlibs/pathoptimization/clearance/ClearanceCalculator.hpp>
NAMED_ABSTRACTPTR(ClearanceCalculator,rwlibs::pathoptimization::ClearanceCalculator)

%include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
NAMED_OWNEDPTR(ClearanceOptimizer,rwlibs::pathoptimization::ClearanceOptimizer)

%include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>
NAMED_OWNEDPTR(MinimumClearanceCalculator,rwlibs::pathoptimization::MinimumClearanceCalculator)

%include  <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
NAMED_OWNEDPTR(PathLengthOptimizer,rwlibs::pathoptimization::PathLengthOptimizer)