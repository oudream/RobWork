%module sdurw_proximitystrategies

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
%}

%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw.i>


%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
%}




class ProximityStrategyFactory
{
public:
    static std::vector<std::string> getCollisionStrategyIDs();

	static rw::core::Ptr<CollisionStrategy> makeDefaultCollisionStrategy();

	static rw::core::Ptr<CollisionStrategy> makeCollisionStrategy(const std::string& id);

    static std::vector<std::string> getDistanceStrategyIDs();

	static rw::core::Ptr<DistanceStrategy> makeDefaultDistanceStrategy();

	static rw::core::Ptr<DistanceStrategy> makeDistanceStrategy(const std::string& id);
};