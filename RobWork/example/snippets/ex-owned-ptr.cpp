#include <rw/common/Ptr.hpp>

using namespace rw::common;

class T {
    public:
        typedef rw::common::Ptr<T> Ptr;
        typedef rw::common::Ptr<const T> CPtr;

        Ptr make()
        {
            return ownedPtr(new T);
        }
};
