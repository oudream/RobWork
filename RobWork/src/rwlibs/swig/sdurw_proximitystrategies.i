%module sdurw_proximitystrategies

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
%}

%include <exception.i>

%import <rwlibs/swig/sdurw.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
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