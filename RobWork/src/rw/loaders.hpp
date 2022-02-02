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
/**
 * @file loaders.hpp
 *
 * this file includes all header files from the loaders namespace
 */

#ifndef RW_LOADERS_HPP_
#define RW_LOADERS_HPP_

#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/colsetup/CollisionSetupLoader.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMPathLoader.hpp>
#include <rw/loaders/dom/DOMPathSaver.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMProximitySetupLoader.hpp>
#include <rw/loaders/dom/DOMTrajectoryLoader.hpp>
#include <rw/loaders/dom/DOMTrajectorySaver.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/loaders/image/RGBLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/rwxml/DependencyGraph.hpp>
#include <rw/loaders/rwxml/MultipleFileIterator.hpp>
#include <rw/loaders/rwxml/XML.hpp>
#include <rw/loaders/rwxml/XMLErrorHandler.hpp>
#include <rw/loaders/rwxml/XMLParser.hpp>
#include <rw/loaders/rwxml/XMLParserUtil.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/loaders/rwxml/XMLRWParser.hpp>
#include <rw/loaders/rwxml/XMLRWPreParser.hpp>
//#include <rw/loaders/TaskLoader.hpp>

#if RW_HAVE_XERCES
#include <rw/loaders/xml/XMLPathLoader.hpp>
#include <rw/loaders/xml/XMLPathSaver.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/loaders/xml/XMLTrajectoryLoader.hpp>
#include <rw/loaders/xml/XMLTrajectorySaver.hpp>
#endif

#endif /* LOADERS_HPP_ */
