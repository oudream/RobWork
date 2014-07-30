/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_
#define RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_

#include <rwlibs/calibration/WorkCellCalibration.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/** 
 * @brief Save function for writing a work cell calibration in an XML format
 */
class XmlCalibrationSaver {
public:
	/**
	 * @brief Saves \bworkcellCalibration to the file \bfileName
	 * @param workcellCalibration [in] workcellCalibration to store
	 * @param fileName [in] Name of the file to which to write.
	 */
	static void save(WorkCellCalibration::Ptr workcellCalibration, std::string fileName);

	/**
	 * @brief Writes \bworkcellCalibration to stream.
	 * @param workcellCalibration [in] workcellCalibration to write
	 * @param ostream [in] Stream to write to
	 */
	static void save(WorkCellCalibration::Ptr workcellCalibration, std::ostream& ostream);
};

/* @} */

}
}

#endif /* RWLIBS_CALIBRATION_XMLCALIBRATIONSAVER_HPP_ */
