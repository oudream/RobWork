/*
 * GradientOptimizer.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: dagothar
 */

#include "GradientOptimizer.hpp"

#include "FunctionWithNumericalDerivative.hpp"

using namespace rw::core;
using namespace rw::math;

namespace rwlibs { namespace optimization {

    GradientOptimizer::GradientOptimizer(
        typename FunctionType::Ptr function,
        typename NumericalDerivativeType::Ptr numericalDerivative) :
        Optimizer(NULL),
        _numericalDerivative(numericalDerivative) {
        setFunction(function);
    }

    void GradientOptimizer::setFunction(typename FunctionType::Ptr function) {
        if(!dynamic_cast<Function1DiffType*>(function.get())) {
            RW_WARN("GradientOptimizer received a function without derivative implementation. "
                    "Wrapping function in numerical derivative object.");

            function = new FunctionWithNumericalDerivative<ResultType, VectorType, GradientType>(
                function, _numericalDerivative);
        }

        Optimizer::setFunction(function);
    }

}}    // namespace rwlibs::optimization
