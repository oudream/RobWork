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


#ifndef RW_COMMON_PROPERTYTYPE_HPP
#define RW_COMMON_PROPERTYTYPE_HPP

/**
   @file PropertyType.hpp
*/

#include <string>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/trajectory/Path.hpp>

namespace rw { namespace common {

    //Forward declaration of PropertyMap
    class PropertyMap;

    /** @addtogroup common */
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
    	/**
    	 * @brief Predefined types
    	 */
        typedef enum {
        		Unknown = -1,    /** Unknown type */
                PropertyMap = 0,/** PropertyMap */
                String,         /** std::string */
                Float,          /** float */
                Double,         /** double */
                Int,            /** int */
                Bool,           /** bool */
                Vector3D,       /** rw::math::Vector3D<> */
                Vector2D,       /** rw::math::Vector2D<> */
                Q,              /** rw::math::Q */
                Transform3D,    /** rw::math::Transform3D<> */
                Rotation3D,     /** rw::math::Rotation3D<> */
                RPY,            /** rw::math::RPY<> */
                EAA,            /** rw::math::EAA<> */
                Quaternion,     /** rw::math::Quaternoin<> */
                Rotation2D,     /** rw::math::Rotation2D<> */
                VelocityScrew6D,/** rw::math::VelocityScrew6D<> */
                QPath,          /** rw::trajectory::QPath */
                Transform3DPath,  /** rw::trajectory::Transform3DPath */
                User            /** First user defined type. Returned by first call to PropertyType::getNewId() */
                } Types;

        /**
         * brief Constructs PropertyType with type UNKNOWN
         */
        PropertyType() : _id(Unknown) {}

        /**
         * @brief Construct PropertyType with the specified type
         *
         * @param id [in] either one of the predefined types or a user defined
         * type, generated by getNewID().
         */
        PropertyType(int id) : _id(id) {}

        /**
         * @brief Constructs a new ID for a property type. The ID will be unique
         * @return the id
         */
        static int getNewID();

        /**
         * @brief Returns id of the type
         * @return the id
         */
        int getId() const { return _id; }

        /**
           @brief The type of a value T resolved at compile time.
        */
        static PropertyType getType(const rw::common::PropertyMap&) { return PropertyType(PropertyMap); }
        static PropertyType getType(const std::string&) { return PropertyType(String); }
        static PropertyType getType(float) { return PropertyType(Float); }
        static PropertyType getType(double) { return PropertyType(Double); }
        static PropertyType getType(int) { return PropertyType(Int); }
        static PropertyType getType(bool) { return PropertyType(Bool); }
        static PropertyType getType(const rw::math::Vector3D<>&) { return PropertyType(Vector3D); }
        static PropertyType getType(const rw::math::Vector2D<>&) { return PropertyType(Vector2D); }
        static PropertyType getType(const rw::math::Q&) { return PropertyType(Q); }
        static PropertyType getType(const rw::math::Transform3D<>&) { return PropertyType(Transform3D); }
        static PropertyType getType(const rw::math::Rotation3D<>&) { return PropertyType(Rotation3D); }
        static PropertyType getType(const rw::math::RPY<>&) { return PropertyType(RPY); }
        static PropertyType getType(const rw::math::EAA<>&) { return PropertyType(EAA); }
        static PropertyType getType(const rw::math::Quaternion<>&) { return PropertyType(Quaternion); }
        static PropertyType getType(const rw::math::Rotation2D<>&) { return PropertyType(Rotation2D); }
        static PropertyType getType(const rw::math::VelocityScrew6D<>&) { return PropertyType(VelocityScrew6D); }
        static PropertyType getType(const rw::trajectory::QPath&) { return PropertyType(QPath); }
        static PropertyType getType(const rw::trajectory::Transform3DPath&) { return PropertyType(Transform3DPath); }

        template <class T>
        static PropertyType getType(const T&) { return PropertyType(Unknown); }

    private:
        int _id;
        static int _NextID;
    };

    /** @} */

}} // end namespaces

#endif // end include guard
