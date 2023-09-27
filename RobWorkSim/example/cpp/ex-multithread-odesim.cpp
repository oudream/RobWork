#include <rw/common/CodeTimer.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactStrategyDMS.hpp>
#include <rwsim/contacts/ThreadedContactDetector.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

#include <boost/filesystem.hpp>

using rw::core::ownedPtr;
using rw::kinematics::State;
using rw::math::Q;

using rwsim::loaders::DynamicWorkCellLoader;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsim::contacts;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;

std::string engineID = "ODE";

std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& list) {
    for(std::string s : list) { os << s << ", "; }
    return os;
}

DynamicSimulator::Ptr simulator_NativeContact(DynamicWorkCell::Ptr dwc) {
    PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID, dwc);
    if(engine.isNull())
        RW_THROW("Engine: " << engineID << " Not Found. Available is: "
                            << PhysicsEngine::Factory::getEngineIDs());
    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));

    return simulator;
}

DynamicSimulator::Ptr simulator_ContactStrategyPQP(DynamicWorkCell::Ptr dwc) {
    PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID, dwc);
    if(engine.isNull())
        RW_THROW("Engine: " << engineID << " Not Found. Available is: "
                            << PhysicsEngine::Factory::getEngineIDs());

    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));
    ContactDetector::Ptr cd         = rw::core::ownedPtr(new ContactDetector(dwc->getWorkCell()));
    ContactStrategy::Ptr cs         = rw::core::ownedPtr(
        new ContactStrategyDMS<rwlibs::proximitystrategies::ProximityStrategyPQP>());

    cs->setPropertyMap(dwc->getEngineSettings());
    cd->addContactStrategy(cs);
    engine->setContactDetector(cd);

    return simulator;
}

DynamicSimulator::Ptr simulator_ContactStrategyFCL(DynamicWorkCell::Ptr dwc) {
    PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID, dwc);
    if(engine.isNull())
        RW_THROW("Engine: " << engineID << " Not Found. Available is: "
                            << PhysicsEngine::Factory::getEngineIDs());

    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));
    ContactDetector::Ptr cd         = rw::core::ownedPtr(new ContactDetector(dwc->getWorkCell()));
    ContactStrategy::Ptr cs         = rw::core::ownedPtr(
        new ContactStrategyDMS<rwlibs::proximitystrategies::ProximityStrategyFCL>());

    cs->setPropertyMap(dwc->getEngineSettings());
    cd->addContactStrategy(cs);
    engine->setContactDetector(cd);

    return simulator;
}

DynamicSimulator::Ptr simulator_ContactStrategyThread(DynamicWorkCell::Ptr dwc) {
    PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID, dwc);
    if(engine.isNull())
        RW_THROW("Engine: " << engineID << " Not Found. Available is: "
                            << PhysicsEngine::Factory::getEngineIDs());

    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));
    ThreadedContactDetector::Ptr cd = rw::core::ownedPtr(new ThreadedContactDetector(dwc));
    engine->setContactDetector(cd);

    return simulator;
}

DynamicSimulator::Ptr simulator_ContactStrategyThreadSingle(DynamicWorkCell::Ptr dwc) {
    PhysicsEngine::Ptr engine = PhysicsEngine::Factory::makePhysicsEngine(engineID, dwc);
    if(engine.isNull())
        RW_THROW("Engine: " << engineID << " Not Found. Available is: "
                            << PhysicsEngine::Factory::getEngineIDs());

    DynamicSimulator::Ptr simulator = ownedPtr(new DynamicSimulator(dwc, engine));
    ThreadedContactDetector::Ptr cd = rw::core::ownedPtr(new ThreadedContactDetector(dwc, 1));
    engine->setContactDetector(cd);
    return simulator;
}

int main(int argc, char** argv) {
    boost::filesystem::path here(__FILE__);
    boost::filesystem::path dwcFile = here.parent_path().parent_path().parent_path() / "gtest" /
                                      "testfiles" / "scene" / "simple" / "box_of_cones.dwc.xml";

    for(int i = 1; i < argc; i++) {
        if(argv[i] == std::string("-e")) { engineID = argv[++i]; }
    }

    if(!boost::filesystem::exists(dwcFile)) {
        std::cout << "Could not find dwc: " << dwcFile << std::endl;
        return 1;
    }

    // Setup ẂorkCell
    DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwcFile.string());
    Body::Ptr box            = dwc->findBody("Box");

    // create the simulator instance

    TimedStatePath path;

    rw::common::CodeTimer timer("Simulation");

    for(auto simInfo :
        (std::vector<
            std::pair<std::string, std::function<DynamicSimulator::Ptr(DynamicWorkCell::Ptr)>>>){
            {"NativePQP",            simulator_NativeContact              },
            {"StrategyPQP",          simulator_ContactStrategyPQP         },
            {"StrategyFCL",          simulator_ContactStrategyFCL         },
            {"StrategyThreadSingle", simulator_ContactStrategyThreadSingle},
            {"StrategyThread",       simulator_ContactStrategyThread      }
    }) {
        DynamicSimulator::Ptr simulator = simInfo.second(dwc);
        State state                     = dwc->getWorkCell()->getDefaultState();

        simulator->init(state);
        simulator->reset(state);
        rw::common::CodeTimer simTimer(simInfo.first);
        double t = 0;
        std::cout << "Simulating: " << simInfo.first << std::endl;
        while(t < 2.0) {
            simulator->setTarget(box,
                                 rw::math::VelocityScrew6D<double>(0,
                                                                   0,
                                                                   sin(rw::math::Pi * t * 50) * 0.5,
                                                                   sin(rw::math::Pi * t * 10) * 0.3,
                                                                   cos(rw::math::Pi * t * 10) * 0.3,
                                                                   0));
            simulator->step(0.001);
            t = simulator->getTime();
            if(uint32_t(t * 1000) % 33 == 0) {
                std::cout << t / 2.0 * 100 << "%      \r" << std::flush;
                path.push_back(TimedState(t, simulator->getState()));
            }
        }
    }
    timer.stop();
    timer.getRepport();

    rw::loaders::PathLoader::storeTimedStatePath(*(dwc->getWorkCell()), path, "./Path.rwplay");

    return 0;
}
