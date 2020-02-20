#include <rwhw/dockwelder/DockWelder.hpp>

#include <iostream>
#include <string>

using namespace rw;
using namespace rwhw;

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <ServerIP>" << std::endl;
        exit (1);
    }

    try {
        std::string ip = argv[2];
        DockWelder dock_welder;
        dock_welder.openConnection(ip);
    }
    catch (std::exception const& e) {
        std::cout << "Can't open connection: " << e.what() << std::endl;
    }

    return 0;
}