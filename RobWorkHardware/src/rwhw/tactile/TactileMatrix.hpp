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

#ifndef RWHW_TACTILEMATRIX_HPP
#define RWHW_TACTILEMATRIX_HPP

#include <boost/numeric/ublas/matrix.hpp>
#include <iostream>

namespace rwhw {

/** @addtogroup tactile */
/* @{ */

class TactileMatrix
{
  public:
    TactileMatrix (std::size_t columns, std::size_t rows) : _data (columns, rows)
    {
        // std::cout << "Tactile ( " << columns << ":" << rows << ")" << std::endl;
        _data = boost::numeric::ublas::zero_matrix< float > (columns, rows);
    }

    virtual ~TactileMatrix () {}

    void set (std::size_t column, std::size_t row, float val)
    {
        // std::cout << "Set :" << column << ";"<< row << ";"<< val << std::endl;
        _data (column, row) = val;
    }

    float get (std::size_t column, std::size_t row) { return _data (column, row); }

    const boost::numeric::ublas::matrix< float >& getMatrix () const { return _data; }

    std::string toString () const
    {
        std::ostringstream ostr;
        for (size_t row = 0; row < _data.size2 (); row++) {
            ostr << row * 3.4 + 1.7 << ";";
            for (size_t col = 0; col < _data.size1 () - 1; col++) {
                ostr << _data (col, row) << ";";
            }
            ostr << _data (_data.size1 () - 1, row) << std::endl;
        }
        return ostr.str ();
    }

  private:
    boost::numeric::ublas::matrix< float > _data;
};
/* @} */

}    // namespace rwhw

#endif /*RWHW_TACTILEMATRIX_HPP*/
