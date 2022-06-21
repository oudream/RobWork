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

#ifndef RWLIBS_CSG_ENGNE_CSGJSENGINE_HPP_
#define RWLIBS_CSG_ENGNE_CSGJSENGINE_HPP_
#include <rw/core/Plugin.hpp>
#include <rw/geometry/CSGEngine.hpp>

namespace rwlibs { namespace csg { namespace engines {

    class CSGJSEngine : public rw::geometry::CSGEngine
    {
      public:
        using Ptr = rw::core::Ptr< CSGJSEngine >;
        /**
         * @brief Construct a new CSGJSEngine object
         *
         */
        CSGJSEngine () {}
        /**
         * @brief get the String id of the engine
         * @return std::string
         */
        std::string getID () const { return "CSGJS"; }

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        rw::geometry::TriMeshData::Ptr Union (rw::geometry::TriMeshData::Ptr m1,
                                              rw::geometry::TriMeshData::Ptr m2) const;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        rw::geometry::TriMeshData::Ptr Difference (rw::geometry::TriMeshData::Ptr m1,
                                                   rw::geometry::TriMeshData::Ptr m2) const;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        rw::geometry::TriMeshData::Ptr Intersection (rw::geometry::TriMeshData::Ptr m1,
                                                     rw::geometry::TriMeshData::Ptr m2) const;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        rw::geometry::TriMeshData::Ptr
        SymmetricDifference (rw::geometry::TriMeshData::Ptr m1,
                             rw::geometry::TriMeshData::Ptr m2) const;
    };

    /**
     * @brief A plugin providing CSGJSEngine for RobWork.
     */
    class CSGEJSEnginePlugin : public rw::core::Plugin
    {
      public:
        //! @brief Construct new plugin
        CSGEJSEnginePlugin ();

        //! @brief Destructor
        virtual ~CSGEJSEnginePlugin ();

        //! @copydoc rw::core::Plugin::getExtensionDescriptors
        std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ();

        //! @copydoc rw::core::Plugin::makeExtension
        rw::core::Ptr< rw::core::Extension > makeExtension (const std::string& id);

        //! @brief Register the plugins extensions in the rw::core::ExtensionRegistry.
        static void registerPlugin ();
    };

}}}    // namespace rwlibs::csg::engines

#endif