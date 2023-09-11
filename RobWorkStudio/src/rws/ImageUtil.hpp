/********************************************************************************
 * Copyright 2011 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWS_IMAGEUTIL_HPP_
#define RWS_IMAGEUTIL_HPP_

#include <rw/sensor/Image.hpp>

class QImage;

namespace rws {

/**
 * utility functions (mostly for conversion) for using thw RobWork Image class in Qt contexts
 */
class ImageUtil
{
  public:
    /**
     * @brief Convert Qt image to RobWork image format.
     * @param srcimg [in] the QImage.
     * @return a new equivalent RobWork image.
     */
    static rw::sensor::Image::Ptr toRwImage(const QImage& srcimg);

    /**
     * @brief Convert Qt image to RobWork image format.
     * @param srcimg [in] the QImage.
     * @param dstimg [out] the RobWork image.
     */
    static void toRwImage(const QImage& srcimg, rw::sensor::Image& dstimg);

    /**
     * @brief Convert RobWork image to Qt image format.
     * @param srcimg [in] the RobWork image.
     * @return a new equivalent QImage.
     */
    static QImage* toQtImage(const rw::sensor::Image& srcimg);
};

}    // namespace rws
#endif /* IMAGEUTIL_HPP_ */
