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

#ifndef RW_MATH_TRANSFORM2D_HPP
#define RW_MATH_TRANSFORM2D_HPP

/**
 * @file Transform2D.hpp
 */

#if !defined(SWIG)
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Vector2D.hpp>

#include <cassert>
#endif 

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A 3x3 homogeneous transform matrix @f$ \mathbf{T}\in SE(2) @f$
     *
     * @f$
     * \mathbf{T} =
     * \left[
     *  \begin{array}{cc}
     *  \mathbf{R} & \mathbf{d} \\
     *  \begin{array}{cc}0 & 0\end{array} & 1
     *  \end{array}
     * \right]
     * @f$
     *
     */

    template<class T>
    struct Transform2DIdentity;

    template< class T = double > class Transform2D
    {
      public:
        //! Value type.
        typedef T value_type;

        /**
         * @brief Default Constructor.
         *
         * Initializes with 0 translation and Identity matrix as rotation
         */
        Transform2D () : _d (), _R (Rotation2D< T >::identity ()) {}

        /**
         * @brief Constructs a homogeneous transform
         * @param d [in] @f$ \mathbf{d} @f$ A 2x1 translation vector
         * @param R [in] @f$ \mathbf{R} @f$ A 2x2 rotation matrix
         */
        Transform2D (const Vector2D< T >& d, const Rotation2D< T >& R) : _d (d), _R (R) {}

        /**
         * @brief Constructs the identity transform
         * @return the identity transform
         *
         * @f$
         * \mathbf{T} =
         * \left[
         * \begin{array}{ccc}
         * 1 & 0 & 0 \\
         * 0 & 1 & 0 \\
         * 0 & 0 & 1 \\
         * \end{array}
         * \right]
         * @f$
         */
        static const Transform2D& identity ()
        {
            return Transform2DIdentity<T>::identity();
        }

        /**
         * @brief Returns matrix element reference
         * @param row [in] row, row must be @f$ < 2 @f$
         * @param col [in] col, col must be @f$ < 3 @f$
         * @return reference to matrix element
         */
        T& operator() (std::size_t row, std::size_t col)
        {
            assert (row < 2);
            assert (col < 3);
            if (row < 2 && col < 2)
                return _R (row, col);
            else
                return _d (row);
        }

        /**
         * @brief Returns const matrix element reference
         * @param row [in] row, row must be @f$ < 2 @f$
         * @param col [in] col, col must be @f$ < 3 @f$
         * @return const reference to matrix element
         */
        const T& operator() (std::size_t row, std::size_t col) const
        {
            assert (row < 2);
            assert (col < 3);
            if (row < 2 && col < 2)
                return _R (row, col);
            else
                return _d (row);
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true only if all elements are equal.
         *
         * @param rhs [in] Transform to compare with
         * @return True if equal.
         */
        bool operator==(const Transform2D<T>& rhs) const
        {
            return (R() == rhs.R()) && (P() == rhs.P());
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true if any of the elements are different.
         *
         * @param rhs [in] Transform to compare with
         * @return True if not equal.
         */
        bool operator!=(const Transform2D<T>& rhs) const
        {
            return !(*this == rhs);
        }

        /**
           @brief Calculates @f$ \robabx{a}{c}{\mathbf{T}} =
           \robabx{a}{b}{\mathbf{T}} \robabx{b}{c}{\mathbf{T}} @f$
           where this object is @f$ \robabx{a}{b}{\mathbf{T}} @f$.

           @param bTc [in] @f$ \robabx{b}{c}{\mathbf{T}} @f$
           @return @f$ \robabx{a}{c}{\mathbf{T}} @f$

           @f$
           \robabx{a}{c}{\mathbf{T}} =
           \left[
           \begin{array}{cc}
           \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{R}} &
           \robabx{a}{b}{\mathbf{d}} + \robabx{a}{b}{\mathbf{R}}\robabx{b}{c}{\mathbf{d}} \\
           \begin{array}{cc} 0 & 0 \end{array} & 1
           \end{array}
           \right]
           @f$
        */
        const Transform2D operator* (const Transform2D& bTc) const
        {
            return Transform2D (this->_d + this->_R * bTc._d, this->_R * bTc._R);
        }

        /**
           @brief Calculates @f$ \robax{a}{\mathbf{p}} =
           \robabx{a}{b}{\mathbf{T}} \robax{b}{\mathbf{p}} \f$ thus transforming
           point @f$ \mathbf{p} @f$ from frame @f$ b @f$ to frame @f$ a @f$.
           This object is @f$ \robabx{a}{c}{\mathbf{T}} @f$.

           @param bP [in] @f$ \robax{b}{\mathbf{p}} @f$
           @return @f$ \robax{a}{\mathbf{p}} @f$
        */
        const Vector2D< T > operator* (const Vector2D< T >& bP) const
        {
            return this->_R * bP + this->_d;
        }

        /**
         * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{R} @f$
         */
        Rotation2D< T >& R () { return _R; }

        /**
         * @brief Gets the rotation part @f$ \mathbf{R} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{R} @f$
         */
        const Rotation2D< T >& R () const { return _R; }

        /**
         * \brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
         * \return @f$ \mathbf{d} @f$
         */
        Vector2D< T >& P () { return _d; }

        /**
         * @brief Gets the position part @f$ \mathbf{d} @f$ from @f$ \mathbf{T} @f$
         * @return @f$ \mathbf{d} @f$
         */
        const Vector2D< T >& P () const { return _d; }

        /**
         * @brief Outputs transform to stream
         * @param os [in/out] an output stream
         * @param t [in] the transform that is to be sent to the output stream
         * @return os
         */
        friend std::ostream& operator<< (std::ostream& os, const Transform2D< T >& t)
        {
            return os << "Transform2D(" << t.P () << ", " << t.R () << ")";
        }

      private:
        Vector2D< T > _d;
        Rotation2D< T > _R;
    };

    extern const Transform2D<double> Transform2DDoubleIdentity;
    extern const Transform2D<float> Transform2DFloatIdentity;

    template<class T>
    struct Transform2DIdentity {
        static const Transform2D<T>& identity() {
            static const Transform2D<> id (Vector2D< T > (0, 0), Rotation2D< T >::identity ());
            return id;
        }
    };

    template<>
    struct Transform2DIdentity<double> {
        static const Transform2D<double>& identity() {
            return Transform2DDoubleIdentity;
        }
    };

    template<>
    struct Transform2DIdentity<float> {
        static const Transform2D<float>& identity() {
            return Transform2DFloatIdentity;
        }
    };

    /**
     * @brief Cast Transform2D<T> to Transform2D<Q>
     * @param trans [in] Transform2D with type T
     * @return Transform2D with type Q
     */
    template< class Q, class T > const Transform2D< Q > cast (const Transform2D< T >& trans)
    {
        Transform2D< Q > res;
        for (size_t i = 0; i < 2; i++)
            for (size_t j = 0; j < 3; j++)
                res (i, j) = static_cast< Q > (trans (i, j));

        return res;
    }

    /**
     * @brief Calculates @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
     * @relates Transform2D
     * @param aTb [in] the transform matrix @f$ \robabx{a}{b}{\mathbf{T}} @f$
     * @return @f$ \robabx{b}{a}{\mathbf{T}} = \robabx{a}{b}{\mathbf{T}}^{-1} @f$
     *
     * @f$
     * \robabx{a}{b}{\mathbf{T}}^{-1} =
     * \left[
     *  \begin{array}{cc}
     *  \robabx{a}{b}{\mathbf{R}}^{T} & - \robabx{a}{b}{\mathbf{R}}^{T} \robabx{a}{b}{\mathbf{d}} \\
     *  \begin{array}{cc}0 & 0\end{array} & 1
     *  \end{array}
     * \right]
     *
     * @f$
     */
    template< class T > const Transform2D< T > inverse (const Transform2D< T >& aTb)
    {
        return Transform2D< T > (-(inverse (aTb.R ()) * aTb.P ()), inverse (aTb.R ()));
    }

    extern template class rw::math::Transform2D< double >;
    extern template class rw::math::Transform2D< float >;

    using Transform2Dd = Transform2D<double>;
    using Transform2Df = Transform2D<float>;

    /*@}*/

}}    // namespace rw::math

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Transform2D
         */
        template<>
        void write (const rw::math::Transform2D< double >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Transform2D
         */
        template<>
        void write (const rw::math::Transform2D< float >& sobject,
                    rw::common::OutputArchive& oarchive, const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Transform2D
         */
        template<>
        void read (rw::math::Transform2D< double >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Transform2D
         */
        template<>
        void read (rw::math::Transform2D< float >& sobject, rw::common::InputArchive& iarchive,
                   const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

namespace boost {
    namespace serialization {
        /**
         * @brief Boost serialization.
         * @param archive [in] the boost archive to read from or write to.
         * @param transform [in/out] the transformation to read/write.
         * @param version [in] class version (currently version 0).
         * @relatedalso rw::math::Transform2D
         */
        template<class Archive, class T>
        void serialize(Archive& archive, rw::math::Transform2D<T>& transform,
            const unsigned int version)
        {
            archive& transform.P();
            archive& transform.R();
        }
    }
} // end namespaces

#endif    // end include guard
