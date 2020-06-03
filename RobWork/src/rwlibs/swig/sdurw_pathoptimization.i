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
            return new PathLengthOptimizer(constraint, rw::math::MetricFactory::makeEuclidean< Q>());
        }

        PathLengthOptimizer(rw::core::Ptr<CollisionDetector> cd,
                            rw::core::Ptr<Device> dev,
                            rw::core::Ptr< rw::math::Metric<Q> > metric,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, metric );
        }

        PathLengthOptimizer(rw::core::Ptr<PlannerConstraint> constraint,
                            rw::core::Ptr< rw::math::Metric<Q> > metric)
        {
            return new PathLengthOptimizer(*constraint, metric);
        }

        rw::core::Ptr<rw::trajectory::Path<Q> > pathPruning(rw::core::Ptr<rw::trajectory::Path<Q> > path){
            rw::trajectory::Path<Q> res = $self->rwlibs::pathoptimization::PathLengthOptimizer::pathPruning(*path);
            return rw::core::ownedPtr( new rw::trajectory::Path<Q>(res) );
        }
/*
        rw::core::Ptr<rw::trajectory::Path<Q> > shortCut(rw::core::Ptr<rw::trajectory::Path<Q> > path,
                                       size_t cnt,
                                       double time,
                                       double subDivideLength);
*/
        rw::core::Ptr<rw::trajectory::Path<Q> > shortCut(rw::core::Ptr<rw::trajectory::Path<Q> > path){
            rw::trajectory::Path<Q> res = $self->rwlibs::pathoptimization::PathLengthOptimizer::shortCut(*path);
            return rw::core::ownedPtr( new rw::trajectory::Path<Q>(res) );
        }

        rw::core::Ptr<rw::trajectory::Path<Q> > partialShortCut(rw::core::Ptr<rw::trajectory::Path<Q> > path){
            rw::trajectory::Path<Q> res = $self->rwlibs::pathoptimization::PathLengthOptimizer::partialShortCut(*path);
            return rw::core::ownedPtr( new rw::trajectory::Path<Q>(res) );
        }
/*
        rw::core::Ptr<rw::trajectory::Path<Q> > partialShortCut(rw::core::Ptr<rw::trajectory::Path<Q> > path,
                                              size_t cnt,
                                              double time,
                                              double subDivideLength);
                                              */
    }
    PropertyMap& getPropertyMap();

};