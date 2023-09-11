/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_MATHEMATICA_OUTPUTNAMEPACKET_HPP_
#define RWLIBS_MATHEMATICA_OUTPUTNAMEPACKET_HPP_

/**
 * @file OutputNamePacket.hpp
 *
 * \copydoc rwlibs::mathematica::OutputNamePacket
 */

#include <rw/core/Ptr.hpp>
#include <rwlibs/mathematica/Mathematica.hpp>

namespace rwlibs { namespace mathematica {
    //! @addtogroup mathematica

    //! @{
    //! @brief A Mathematica WSTP %OutputNamePacket.
    class OutputNamePacket : public Mathematica::Packet
    {
      public:
        //! @brief Smart pointer type.
        typedef rw::core::Ptr<OutputNamePacket> Ptr;

        /**
         * @brief Construct new packet.
         * @param string [in] the string argument.
         */
        OutputNamePacket(const Mathematica::String& string);

        //! @brief Destructor.
        virtual ~OutputNamePacket();

        /**
         * @brief Get the string stored in the packet.
         * @return the string.
         */
        const Mathematica::String& string();

        //! @copydoc Mathematica::FunctionBase::getArguments
        std::list<rw::core::Ptr<const Mathematica::Expression>> getArguments() const;

        //! @copydoc Mathematica::Expression::clone
        Mathematica::Expression::Ptr clone() const;

      private:
        const Mathematica::String _string;
    };
    //! @}
}}     // namespace rwlibs::mathematica
#endif /* RWLIBS_MATHEMATICA_OUTPUTNAMEPACKET_HPP_ */
