%module sdurw_pathoptimization

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
using rw::math::Metric;
using rw::trajectory::Path;
%}

%include <exception.i>
%import <rwlibs/swig/sdurw.i>


%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
%}

class PathLengthOptimizer
{
public:

    %extend {

        PathLengthOptimizer(rw::core::Ptr<CollisionDetector> cd,
                            rw::core::Ptr<Device> dev,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, rw::math::MetricFactory::makeEuclidean< rw::math::Q>());
        }

        PathLengthOptimizer(rw::core::Ptr<CollisionDetector> cd,
                            rw::core::Ptr<Device> dev,
                            rw::core::Ptr<Metric<rw::math::Q> > metric,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, metric );
        }

        PathLengthOptimizer(rw::core::Ptr<PlannerConstraint> constraint,
                            rw::core::Ptr<Metric<rw::math::Q> > metric)
        {
            return new PathLengthOptimizer(*constraint, metric);
        }

        rw::core::Ptr<Path<rw::math::Q> > pathPruning(rw::core::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::pathPruning(*path);
            return rw::core::ownedPtr( new PathQ(res) );
        }
/*
        rw::core::Ptr<Path<rw::math::Q> > shortCut(rw::core::Ptr<Path<rw::math::Q> > path,
                                       size_t cnt,
                                       double time,
                                       double subDivideLength);
*/
        rw::core::Ptr<Path<rw::math::Q> > shortCut(rw::core::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::shortCut(*path);
            return rw::core::ownedPtr( new PathQ(res) );
        }

        rw::core::Ptr<Path<rw::math::Q> > partialShortCut(rw::core::Ptr<Path<rw::math::Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::partialShortCut(*path);
            return rw::core::ownedPtr( new PathQ(res) );
        }
/*
        rw::core::Ptr<Path<rw::math::Q> > partialShortCut(rw::core::Ptr<Path<rw::math::Q> > path,
                                              size_t cnt,
                                              double time,
                                              double subDivideLength);
                                              */
    }
    PropertyMap& getPropertyMap();

};