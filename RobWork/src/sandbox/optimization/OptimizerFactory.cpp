/*
 * OptimizerFactory.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: dagothar
 */

#include "OptimizerFactory.hpp"

#include "BFGSOptimizer.hpp"
#include "DownhillOptimizer.hpp"
#include "GoldenSectionLineSearch.hpp"
#include "LineSearch.hpp"

namespace rwlibs { namespace optimization {

    using namespace rw::core;

    Optimizer::Ptr OptimizerFactory::makeOptimizer(typename FunctionType::Ptr function) {
        return ownedPtr(new DownhillOptimizer(function, 1.0));
    }

    Optimizer::Ptr OptimizerFactory::makeDirectOptimizer(typename FunctionType::Ptr function) {
        return ownedPtr(new DownhillOptimizer(function, 1.0));
    }

    Optimizer::Ptr OptimizerFactory::makeGradientOptimizer(typename FunctionType::Ptr function) {
        return ownedPtr(new BFGSOptimizer(function, 1.0));
    }

    LineSearch::Ptr OptimizerFactory::makeLineSearch(typename FunctionType::Ptr function,
                                                     VectorType direction, double leftBracket,
                                                     double rightBracket) {
        return ownedPtr(
            new GoldenSectionLineSearch(function, direction, leftBracket, rightBracket));
    }

}}    // namespace rwlibs::optimization
