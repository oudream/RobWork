#include <rwhw/universalrobots_rtde/URRTDE.hpp>

#include <iostream>

using namespace rw;
using namespace rwhw;

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <robotIP>" << std::endl;
        exit (1);
    }

    try {
        URRTDE ur_rtde (argv[1]);
    }
    catch (std::exception const& e) {
        std::cout << "Can't open connection: " << e.what() << std::endl;
    }

    return 0;
}