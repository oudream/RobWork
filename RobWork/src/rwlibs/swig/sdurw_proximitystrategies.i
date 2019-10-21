%module sdurw_proximitystrategies

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>

using namespace rwlibs::swig;
%}

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

	static rw::common::Ptr<CollisionStrategy> makeDefaultCollisionStrategy();

	static rw::common::Ptr<CollisionStrategy> makeCollisionStrategy(const std::string& id);

    static std::vector<std::string> getDistanceStrategyIDs();

	static rw::common::Ptr<DistanceStrategy> makeDefaultDistanceStrategy();

	static rw::common::Ptr<DistanceStrategy> makeDistanceStrategy(const std::string& id);
};