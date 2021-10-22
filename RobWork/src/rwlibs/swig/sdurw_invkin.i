%module sdurw_invkin

%include <rwlibs/swig/swig_macros.i>

%include <stl.i>
%include <std_vector.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/ext_i/std.i>
%{
	#include<rw/models/CompositeDevice.hpp>
	#include<rw/models/CompositeJointDevice.hpp>
	#include<rw/models/SE3Device.hpp>
	#include <rw/models/MobileDevice.hpp>
	#include <rw/geometry/IndexedTriangle.hpp>
	#include <rw/geometry/IndexedTriMesh.hpp>
	#include <rw/models/UniversalJoint.hpp>
	#include <rw/models/RevoluteJoint.hpp>
	#include <rw/models/SphericalJoint.hpp>

	#include <rw/models/PrismaticUniversalJoint.hpp>
	#include <rw/models/PrismaticSphericalJoint.hpp>
	#include <rw/models/VirtualJoint.hpp>
	#include <rw/models/DependentJoint.hpp>
	#include <rw/models/DependentRevoluteJoint.hpp>
	#include <rw/models/DependentPrismaticJoint.hpp>
	#include <rw/models/PrismaticJoint.hpp>
	#include <rw/kinematics/FixedFrame.hpp>
	#include <rw/kinematics/MovableFrame.hpp>

	#include <rw/models/DHParameterSet.hpp>

%}


%pragma(java) jniclassimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
	import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
%}
%pragma(java) moduleimports=%{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
	import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
%}
%typemap(javaimports) SWIGTYPE %{
    import org.robwork.sdurw_core.*;
    import org.robwork.sdurw_math.*;
	import org.robwork.sdurw_models.*;
	import org.robwork.sdurw_kinematics.*;
%}

%{
	#include<rw/invkin/InvKinSolver.hpp>
%}
%include <rw/invkin/InvKinSolver.hpp>
%template (InvKinSolverPtr) rw::core::Ptr<rw::invkin::InvKinSolver>;

%{
	#include<rw/invkin/AmbiguityResolver.hpp>
%}
%include <rw/invkin/AmbiguityResolver.hpp>

%{
	#include<rw/invkin/IterativeIK.hpp>
%}
%include <rw/invkin/IterativeIK.hpp>
NAMED_OWNEDPTR(IterativeIK, rw::invkin::IterativeIK);

%{
	#include<rw/invkin/ClosedFormIK.hpp>
%}
%include <rw/invkin/ClosedFormIK.hpp>

NAMED_OWNEDPTR(ClosedFormIK, rw::invkin::ClosedFormIK);

%{
	#include<rw/invkin/IterativeMultiIK.hpp>
%}
%include <rw/invkin/IterativeMultiIK.hpp>


%{
	#include<rw/invkin/CCDSolver.hpp>
%}
%include <rw/invkin/CCDSolver.hpp>


%{
	#include<rw/invkin/ParallelIKSolver.hpp>
%}
%include <rw/invkin/ParallelIKSolver.hpp>


%{
	#include<rw/invkin/PieperSolver.hpp>
%}
%include <rw/invkin/PieperSolver.hpp>
NAMED_OWNEDPTR(PieperSolver, rw::invkin::PieperSolver);

%{
	#include<rw/invkin/IKMetaSolver.hpp>
%}
%include <rw/invkin/IKMetaSolver.hpp>
NAMED_OWNEDPTR(IKMetaSolver, rw::invkin::IKMetaSolver);


%{
	#include<rw/invkin/JacobianIKSolver.hpp>
%}
%include <rw/invkin/JacobianIKSolver.hpp>
NAMED_OWNEDPTR(JacobianIKSolver, rw::invkin::JacobianIKSolver);

%{
	#include<rw/invkin/JacobianIKSolverM.hpp>
%}
%include <rw/invkin/JacobianIKSolverM.hpp>


%{
	#include<rw/invkin/ClosedFormIKSolverUR.hpp>
%}
%include <rw/invkin/ClosedFormIKSolverUR.hpp>
NAMED_OWNEDPTR(ClosedFormIKSolverUR, rw::invkin::ClosedFormIKSolverUR);


%{
	#include<rw/invkin/ClosedFormIKSolverKukaIIWA.hpp>
%}
%include <rw/invkin/ClosedFormIKSolverKukaIIWA.hpp>
NAMED_OWNEDPTR(ClosedFormIKSolverKukaIIWA, rw::invkin::ClosedFormIKSolverKukaIIWA);
