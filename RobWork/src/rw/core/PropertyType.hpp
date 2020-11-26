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

#ifndef RW_CORE_PROPERTYTYPE_HPP
#define RW_CORE_PROPERTYTYPE_HPP

/**
 * @file PropertyType.hpp
 * @copydoc PropertyType.hpp
 */
#if !defined(SWIG)
#include <rw/core/math_fwd.hpp>

#include <string>
#include <vector>
#endif

namespace rw { namespace core {
    template< class T > class Ptr;
}}    // namespace rw::core
namespace rw { namespace trajectory {
    template< class T > class Path;
}}    // namespace rw::trajectory

namespace rw { namespace core {

    // Forward declaration of PropertyMap
    class PropertyMap;
    // Forward declaration of PropertyValueBase
    class PropertyValueBase;

    /** @addtogroup core */
    /*@{*/

    /**
     * @brief Represents type of a property
     *
     * PropertyType has a number of predefined property types. Besides it generates unique
     * id's for new user defined types
     */
    class PropertyType
    {
      public:
        //! @brief Predefined types
        typedef enum {
            Unknown     = -1,         //!< Unknown type
            PropertyMap = 0,          //!< PropertyMap
            PropertyMapPtr,           //!< PropertyMap::Ptr
            PropertyValueBasePtrList, //!< std::vector<rw::core::PropertyValueBase::Ptr>
            String,                   //!< std::string
            Float,                    //!< float
            Double,                   //!< double
            Int,                      //!< int
            Bool,                     //!< bool
            Vector3D,                 //!< rw::math::Vector3D
            Vector2D,                 //!< rw::math::Vector2D
            Q,                        //!< rw::math::Q
            Transform3D,              //!< rw::math::Transform3D
            Rotation3D,               //!< rw::math::Rotation3D
            RPY,                      //!< rw::math::RPY
            EAA,                      //!< rw::math::EAA
            Quaternion,               //!< rw::math::Quaternion
            Rotation2D,               //!< rw::math::Rotation2D
            VelocityScrew6D,          //!< rw::math::VelocityScrew6D
            QPath,                    //!< rw::trajectory::QPath
            QPathPtr,                 //!< rw::trajectory::QPath::Ptr
            Transform3DPath,          //!< rw::trajectory::Transform3DPath
            Transform3DPathPtr,       //!< rw::trajectory::Transform3DPath::Ptr
            StringList,               //!< std::vector<std::string>
            IntList,                  //!< std::vector<int>
            DoubleList,               //!< std::vector<double>
            User = 1024               //!< First user defined type. Returned by first call to
                                      //!< PropertyType::getNewID()
        } Types;

        /**
         * brief Constructs PropertyType with type UNKNOWN
         */
        PropertyType () : _id (Unknown) {}

        /**
         * @brief Construct PropertyType with the specified type
         *
         * @param id [in] either one of the predefined types or a user defined
         * type, generated by getNewID().
         */
        PropertyType (int id) : _id (id) {}

        /**
         * @brief Constructs a new ID for a property type. The ID will be unique
         * @return the id
         */
        static int getNewID ();

        /**
         * @brief Returns id of the type
         * @return the id
         */
        int getId () const { return _id; }

        /**
         * @brief Get the type of a value resolved at compile time.
         * @param value [in] the value to deduct type for.
         * @return the PropertyType corresponding to the type of the given \b value.
         */
        static PropertyType getType (const rw::core::PropertyMap& value)
        {
            return PropertyType (PropertyMap);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::core::Ptr<rw::core::PropertyMap>& value)
        {
            return PropertyType (PropertyMapPtr);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (
                const std::vector<rw::core::Ptr<rw::core::PropertyValueBase> >& value)
        {
            return PropertyType (PropertyValueBasePtrList);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const std::string& value) { return PropertyType (String); }
#if !defined(SWIGLUA)
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (float value) { return PropertyType (Float); }
#endif
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (double value) { return PropertyType (Double); }
#if !defined(SWIGLUA)
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (int value) { return PropertyType (Int); }
#endif
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (bool value) { return PropertyType (Bool); }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Vector3D< double >& value)
        {
            return PropertyType (Vector3D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Vector2D< double >& value)
        {
            return PropertyType (Vector2D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Q& value) { return PropertyType (Q); }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Transform3D< double >& value)
        {
            return PropertyType (Transform3D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Rotation3D< double >& value)
        {
            return PropertyType (Rotation3D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::RPY< double >& value)
        {
            return PropertyType (RPY);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::EAA< double >& value)
        {
            return PropertyType (EAA);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Quaternion< double >& value)
        {
            return PropertyType (Quaternion);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::Rotation2D< double >& value)
        {
            return PropertyType (Rotation2D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::math::VelocityScrew6D< double >& value)
        {
            return PropertyType (VelocityScrew6D);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::trajectory::Path<rw::math::Q>& value)
        {
            return PropertyType (QPath);
        }
#if !defined(SWIG)
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::core::Ptr<rw::trajectory::Path<rw::math::Q> >& value)
        {
            return PropertyType (QPathPtr);
        }
#endif
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::trajectory::Path< rw::math::Transform3D< double > >& value)
        {
            return PropertyType (Transform3DPath);
        }
#if !defined(SWIG)
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const rw::core::Ptr<rw::trajectory::Path< rw::math::Transform3D< double > > >& value)
        {
            return PropertyType (Transform3DPathPtr);
        }
#endif
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const std::vector< std::string >& value)
        {
            return PropertyType (StringList);
        }

#if !defined(SWIGPYTHON)
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const std::vector< int >& value)
        {
            return PropertyType (IntList);
        }
        //! @copydoc getType(const rw::core::PropertyMap&)
        static PropertyType getType (const std::vector< double >& value)
        {
            return PropertyType (DoubleList);
        }
#endif
        /**
         * @brief Get the type of a generic value T resolved at compile time.
         * @param value [in] the value to deduct type for.
         * @return This method will always return PropertyType::Unknown to indicate that no concrete
         * type exist for the given \b value.
         */
        template< class T > static PropertyType getType (const T& value)
        {
            return PropertyType (Unknown);
        }

      private:
        int _id;
        static int _NextID;
    };

    /** @} */

}}    // namespace rw::core

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    using namespace rw::core;
}}    // namespace rw::common

#endif    // end include guard
