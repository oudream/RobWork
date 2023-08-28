#include <rw/core/Ptr.hpp>

using namespace rw::core;

class T {
    public:
        typedef rw::core::Ptr<T> Ptr;
        typedef rw::core::Ptr<const T> CPtr;

        static Ptr make()
        {
            return ownedPtr(new T);
        }
};
