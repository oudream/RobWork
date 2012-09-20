/*
 * XmlCalibrationSaver.hpp
 *
 *  Created on: Sep 19, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_
#define RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_

#include "SerialDeviceCalibration.hpp"

namespace rwlibs {
namespace calibration {

class XmlCalibrationSaver {
public:
	static void save(SerialDeviceCalibration::Ptr, std::string fileName);
};

}
}

#endif /* RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_ */
