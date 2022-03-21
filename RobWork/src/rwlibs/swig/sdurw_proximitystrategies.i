%module sdurw_proximitystrategies

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/models.hpp>

#include <rw/geometry/IndexedTriMesh.hpp>
using namespace rwlibs::swig;
%}
%include <rwlibs/swig/swig_macros.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_proximity.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_proximity.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_proximity.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_proximity.*;
%}




class ProximityStrategyFactory
{
public:
    static std::vector<std::string> getCollisionStrategyIDs();

	static rw::core::Ptr<rw::proximity::CollisionStrategy> makeDefaultCollisionStrategy();

	static rw::core::Ptr<rw::proximity::CollisionStrategy> makeCollisionStrategy(const std::string& id);

    static std::vector<std::string> getDistanceStrategyIDs();

	static rw::core::Ptr<rw::proximity::DistanceStrategy> makeDefaultDistanceStrategy();

	static rw::core::Ptr<rw::proximity::DistanceStrategy> makeDistanceStrategy(const std::string& id);
};