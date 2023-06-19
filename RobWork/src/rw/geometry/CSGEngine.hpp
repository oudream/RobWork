/********************************************************************************
 * Copyright 2022 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
#ifndef RW_GEOMETRY_CSGENGINE_HPP_
#define RW_GEOMETRY_CSGENGINE_HPP_
#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/Ptr.hpp>

#include <Eigen/Core>

namespace rw { namespace geometry {

    /**
     * @brief a simple data structure for keeping the information on vertices and triangles
     * this is the underlying data stucture of SimpleTriMesh, and should be used through that
     */
    struct TriMeshData
    {
        using Ptr = rw::core::Ptr<TriMeshData>;
        /**
         * @brief Default constructor
         */
        TriMeshData() {}

        /**
         * @brief copy constructor
         */
        TriMeshData(const TriMeshData& data) :
            _triangles(data._triangles), _vertecies(data._vertecies) {}

        /**
         * @brief copy constructor
         */
        TriMeshData(const TriMeshData::Ptr& data) :
            _triangles(data->_triangles), _vertecies(data->_vertecies) {}

        Eigen::Matrix<uint32_t, Eigen::Infinity, 3> _triangles;
        Eigen::Matrix<double, Eigen::Infinity, 3> _vertecies;
    };

    /**
     * @brief An abstact class of the nessesary CSG operations. This is intended to be used in
     * conjuction with SimpleTriMesh
     *
     */

    class CSGEngine
    {
      public:
        using Ptr = rw::core::Ptr<CSGEngine>;

        virtual ~CSGEngine() {}

        /**
         * @brief get the String id of the engine
         * @return std::string
         */
        virtual std::string getID() const = 0;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        virtual TriMeshData::Ptr Union(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const = 0;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        virtual TriMeshData::Ptr Difference(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const = 0;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        virtual TriMeshData::Ptr Intersection(TriMeshData::Ptr m1, TriMeshData::Ptr m2) const = 0;

        /**
         * @brief Create a Union of two TriMeshes
         * @param m1 First TriMesh
         * @param m2 Second TriMesh
         * @return The Resulting TriMesh created as a new Shared pointer
         */
        virtual TriMeshData::Ptr SymmetricDifference(TriMeshData::Ptr m1,
                                                     TriMeshData::Ptr m2) const = 0;

        /**
         * @addtogroup extensionpoints
         * @extensionpoint{rw::geometry::CSGEngine::Factory,rw::loaders::CSGEngine,rw.loaders.CSGEngine}
         */

        /**
         * @brief a factory for CSGEngine. This factory defines an extension point for CSGEngine.
         */
        class Factory : public rw::core::ExtensionPoint<CSGEngine>
        {
          public:
            //! constructor
            Factory() :
                rw::core::ExtensionPoint<CSGEngine>("rw.geometry.CSGEngine", "CSGEngineFactory") {}

            /**
             * @brief Get the First CSGEngine in the list.
             * @return CSGEngine or NULL if no Engine is Found
             */
            static CSGEngine::Ptr getDefaultEngine();

            /**
             * @brief set the default CSGEngine
             * @param engine [in] the engine to use
             */
            static void setDefaultEngine(CSGEngine::Ptr engine);

            /**
             * @brief get a list a valid Engine id's
             * @return std::vector< std::string >
             */
            static std::vector<std::string> getAvailableEngines();

            /**
             * @brief Check if engine is available.
             * @param engine [in] the name of the engine.
             * @return true if available, false otherwise.
             */
            static bool hasEngine(const std::string& engine);

            /**
             * @brief get a specific engine.
             * @param id [in] id of the engine
             * @return Null if engine not found else returns the engine
             */
            static CSGEngine::Ptr getCSGEngine(std::string id);
        };

      protected:
        CSGEngine() {}
        CSGEngine(const CSGEngine&);
    };

}}    // namespace rw::geometry

#endif