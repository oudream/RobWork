#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

using rw::common::TimerUtil;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;

int main (int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " /path/to/robworkdata [Option]" << std::endl;
        std::cout << "Option: "
                  << "-t     for test run" << std::endl;
        exit (1);
    }

    const std::string WC_FILE =
        std::string (argv[1]) + "/scenes/SinglePA10Demo/SinglePA10Demo.wc.xml";
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load (WC_FILE);

    rws::RobWorkStudioApp rwsApp ("");
    rwsApp.start ();

    rws::RobWorkStudio* rwstudio = rwsApp.getRobWorkStudio ();
    rwstudio->setWorkCell (workcell);

    while (rwsApp.isRunning ()) {
        TimerUtil::sleepMs (10);    // Check if running every 10 ms

        if (argc == 3 && std::string (argv[2]) == std::string("-t")) {    // Quit if test run
            TimerUtil::sleepMs (1000);          // wait for full startup
            rwsApp.close ();                    // Close RWS
        }
    }
    rwsApp.close ();    // Close rws if running
    return 0;
}
