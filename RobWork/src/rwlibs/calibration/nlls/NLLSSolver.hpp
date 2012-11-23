/*
 * NLLSSolver.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSSOLVER_HPP_
#define RWLIBS_CALIBRATION_NLLSSOLVER_HPP_

#include "NLLSIterationLog.hpp"
#include "NLLSSolverLog.hpp"
#include "NLLSSystem.hpp"
#include <Eigen/Core>

namespace rwlibs {
namespace calibration {

class NLLSSolver {
public:
	typedef rw::common::Ptr<NLLSSolver> Ptr;

	virtual const NLLSSolverLog& getLog() const = 0;

	virtual NLLSIterationLog iterate() = 0;

	virtual bool isConverged() const = 0;

	virtual void solve() = 0;

	virtual Eigen::MatrixXd estimateCovarianceMatrix() const = 0;
};

}
}

#endif /* RWLIBS_CALIBRATION_NLLSSOLVER_HPP_ */
