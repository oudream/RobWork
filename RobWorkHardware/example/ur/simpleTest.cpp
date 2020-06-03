
#include <iostream>


#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>

using namespace rwhw;
using namespace rw::common;
using namespace rw::math;


int main(int argc, char** argv)
{
    ProgramOptions poptions("SimpleURTest1", "0.1");
    poptions.initOptions();
    poptions.parse(argc, argv);

    PropertyMap map = poptions.getPropertyMap();

    Log::infoLog() << "Initializing trakstar sensor" << std::endl;

    rwhw::URCallBackInterface _ur;
    rwhw::UniversalRobotsRTLogging _urrt;

    // SETTINGS for communicating with ur 2 on marvin
    std::string ip("192.168.100.4");
    int port = 33334;

    _urrt.connect(ip, 30003);
    //std::cout<<"Transfer Script: "<<scriptFile<<std::endl;
    _ur.connect(ip, 30001);

    _ur.startCommunication(port);
    _urrt.start();


    Q home(6,0,90*Deg2Rad,0,0,0,0);
    _ur.moveQ(home, 10);


	while (true) {
		if (_urrt.hasData()) {
			URRTData data = _urrt.getLastData();
			std::cout<<"Configuration = "<<data.qActual<<std::endl;
		}
		TimerUtil::sleepMs(10);
	}	
    return 0;
}
