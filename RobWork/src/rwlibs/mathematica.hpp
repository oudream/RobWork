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

#ifndef RWLIBS_MATHEMATICA_HPP_
#define RWLIBS_MATHEMATICA_HPP_

/**
 * @file mathematica.hpp
 *
 * @brief Include file for all Mathematica headers.
 */

#include <rwlibs/mathematica/Mathematica.hpp>

// Packets
#include <rwlibs/mathematica/EnterExpressionPacket.hpp>
#include <rwlibs/mathematica/EnterTextPacket.hpp>
#include <rwlibs/mathematica/EvaluatePacket.hpp>
#include <rwlibs/mathematica/InputNamePacket.hpp>
#include <rwlibs/mathematica/MessagePacket.hpp>
#include <rwlibs/mathematica/OutputNamePacket.hpp>
#include <rwlibs/mathematica/ReturnExpressionPacket.hpp>
#include <rwlibs/mathematica/ReturnPacket.hpp>
#include <rwlibs/mathematica/ReturnTextPacket.hpp>
#include <rwlibs/mathematica/TextPacket.hpp>

// Functions
#include <rwlibs/mathematica/FactorInteger.hpp>
#include <rwlibs/mathematica/Image.hpp>
#include <rwlibs/mathematica/List.hpp>
#include <rwlibs/mathematica/ListPlot.hpp>
#include <rwlibs/mathematica/RawArray.hpp>
#include <rwlibs/mathematica/Rule.hpp>
#include <rwlibs/mathematica/ToExpression.hpp>

#endif /* RWLIBS_MATHEMATICA_HPP_ */
