
#ifndef RW_CORE_ANY_HPP
#define RW_CORE_ANY_HPP

#include <rw/core/Ptr.hpp>

#include <boost/shared_ptr.hpp>
#include <typeinfo>

namespace rw { namespace core {

    //! @brief Smart pointer that can point to any type, and optionally takes ownership of the
    //! object pointer.
    class AnyPtr
    {
      public:
        //! @brief Construct empty null pointer.
        AnyPtr () : content (0) {}

        /**
         * @brief constructor - ownership of pointer is taken
         * @param value [in] a raw pointer.
         */
        template< typename ValueType >
        AnyPtr (ValueType* value) : content (new holder< ValueType > (value))
        {}

        /**
         * @brief constructor - shares ownership.
         * @param value [in] a shared pointer.
         */
        template< typename ValueType >
        AnyPtr (const boost::shared_ptr< ValueType >& value) :
            content (new holder< ValueType > (value))
        {}

        /**
         * @brief Construct from Ptr - shares ownership.
         * @param value [in] a smart pointer.
         */
        template< typename ValueType >
        AnyPtr (const rw::core::Ptr< ValueType >& value) :
            content (new holder< ValueType > (value))
        {}

        /**
         * @brief Copy constructor - ownership is shared.
         * @param other [in] other AnyPtr object.
         */
        AnyPtr (const AnyPtr& other) : content (other.content ? other.content->clone () : 0) {}

        //! @brief Destructor.
        ~AnyPtr ()
        {
            if (content != 0)
                delete content;
        }

        /**
         * @brief Cast to a specific smart pointer type.
         * @return a Ptr object pointing to object if cast success, otherwise a NULL Ptr object is
         * returned.
         */
        template< class S > Ptr< S > cast () const
        {
            S* data = dynamic_cast< S* > ((S*) content->getVoidPtr ());
            if (data == NULL) {
                return Ptr< S > ();
            }

            return ((holder< S >*) content)->_ptr.template cast< S > ();
        }

        /**
         * @brief The pointer stored in the object.
         * @return raw pointer.
         */
        template< class S > S* get () { return dynamic_cast< S* > ((S*) content->getVoidPtr ()); }

        /**
         * @brief Support for implicit conversion to bool.
         */
        operator void* () const { return content->getVoidPtr (); }

        /**
         * @brief Equality operator. This only tests if the pointers to the referenced objects are
         * the same and NOT if the smart pointers are the same.
         * @param p [in] smart pointer to compare with
         * @return true if the referenced objects are the same
         */
        template< class A > bool operator== (const Ptr< A >& p) const
        {
            return content->getVoidPtr () == p.get ();
        }

        /**
         * @brief Tests if the smart pointer points to the same instance as \b p
         * @return true if equal, false otherwise.
         */
        bool operator== (void* p) const { return content->getVoidPtr () == p; }

        /**
         * @brief Check if pointer is null.
         * @return true is the smart pointer is null.
         */
        bool isNull () const { return content->getVoidPtr () == NULL; }

      private:    // types
        class placeholder
        {
          public:    // structors
            virtual ~placeholder () {}

          public:    // queries
            virtual const std::type_info& type () const = 0;

            virtual placeholder* clone () const = 0;

            virtual void* getVoidPtr () const = 0;
        };

        template< typename ValueType > class holder : public placeholder
        {
          public:    // structors
            holder (ValueType* value) : _ptr (value) {}
            holder (boost::shared_ptr< ValueType > value) : _ptr (value) {}
            holder (Ptr< ValueType > value) : _ptr (value) {}

          public:    // queries
            virtual const std::type_info& type () const { return typeid (ValueType); }

            virtual placeholder* clone () const
            {
                return dynamic_cast< placeholder* > (new holder (*this));
            }

            virtual void* getVoidPtr () const { return (void*) _ptr.get (); }

          private:    // intentionally left unimplemented
            holder& operator= (const holder&);

          public:
            Ptr< ValueType > _ptr;
        };

      private:    // representation
        template< typename ValueType > friend ValueType* cast (AnyPtr*);

        placeholder* content;
    };
}}    // namespace rw::core

/**
 * @brief Deprecated namespace since 16/4-2020 for this class
 * @deprecated use rw::core not rw::common
 */
namespace rw { namespace common {
    using namespace rw::core;
}}

#endif
