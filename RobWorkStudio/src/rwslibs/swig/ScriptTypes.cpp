

#include "ScriptTypes.hpp"

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <QApplication>

using namespace rws::swig;
using namespace rwlibs::swig;

rw::core::Ptr< rws::swig::RobWorkStudio > rwstudio_internal;

rws::swig::RobWorkStudio* rws::swig::getRobWorkStudio ()
{
    return rwstudio_internal.get ();
}
rws::swig::RobWorkStudio* rws::swig::getRobWorkStudioFromQt ()
{
    QWidget* rws_w    = NULL;
    QWidgetList all_w = QApplication::allWidgets ();
    for (QWidget* w : all_w) {
        if (w->objectName () == "RobWorkStudio_MainWindow") {
            rws_w = w;
        }
    }

    RobWorkStudio* rws_ = static_cast< RobWorkStudio* > (rws_w);
    return rws_;
}

void rws::swig::setRobWorkStudio (rws::swig::RobWorkStudio* rwstudio)
{
    rwstudio_internal = rwstudio;
}

const State& rws::swig::getState ()
{
    return getRobWorkStudio ()->getState ();
}
void rws::swig::setState (State& state)
{
    return getRobWorkStudio ()->postState (state);
}
rw::core::Ptr< Device > rws::swig::findDevice (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findDevice (name);
}
rw::core::Ptr< JointDevice > rws::swig::findJointDevice (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findDevice< JointDevice > (name);
}
rw::core::Ptr< SerialDevice > rws::swig::findSerialDevice (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findDevice< SerialDevice > (name);
}
rw::core::Ptr< TreeDevice > rws::swig::findTreeDevice (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findDevice< TreeDevice > (name);
}
rw::core::Ptr< ParallelDevice > rws::swig::findParallelDevice (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findDevice< ParallelDevice > (name);
}
Frame* rws::swig::findFrame (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findFrame (name);
}

MovableFrame* rws::swig::findMovableFrame (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findFrame< MovableFrame > (name);
}

FixedFrame* rws::swig::findFixedFrame (const std::string& name)
{
    return getRobWorkStudio ()->getWorkCell ()->findFrame< FixedFrame > (name);
}

void rws::swig::moveTo (MovableFrame* mframe, rw::math::Transform3D<double> wTframe)
{
    State state = getState ();
    mframe->moveTo (wTframe, state);
    setState (state);
}

void rws::swig::moveTo (Frame* frame, MovableFrame* mframe, rw::math::Transform3D<double> wTtcp)
{
    State state                = getState ();
    rw::math::Transform3D<double> tcpTbase      = rw::kinematics::Kinematics::frameTframe (frame, mframe, state);
    rw::math::Transform3D<double> wTbase_target = wTtcp * tcpTbase;
    mframe->moveTo (wTbase_target, state);
    setState (state);
}

void rws::swig::moveTo (const std::string& fname, const std::string& mname, rw::math::Transform3D<double> wTframe)
{
    Frame* fframe        = findFrame (fname);
    MovableFrame* mframe = findMovableFrame (mname);
    moveTo (fframe, mframe, wTframe);
}

static rws::RobWorkStudioApp* robApp = NULL;

rw::core::Ptr< RobWorkStudio > rws::swig::getRobWorkStudioInstance ()
{
    return getRobWorkStudioInstance ("");
}

rw::core::Ptr< RobWorkStudio > rws::swig::getRobWorkStudioInstance (const std::string& args)
{
    // create a thread that start QApplication and
    if (robApp == NULL || !robApp->isRunning ()) {
        robApp = new RobWorkStudioApp (args);
        robApp->start ();
        while (robApp->getRobWorkStudio () == NULL) {
            if (!robApp->isRunning ())
                return NULL;
            rw::common::TimerUtil::sleepMs (100);
        }
        rwstudio_internal = robApp->getRobWorkStudio ();
    }
    return robApp->getRobWorkStudio ();
}

void rws::swig::closeRobWorkStudio ()
{
    robApp->close ();
}

bool rws::swig::isRunning ()
{
    if (robApp == NULL) {
        return false;
    }
    return robApp->isRunning ();
}

rw::math::Q rws::swig::getQ (rw::core::Ptr< rwlibs::swig::Device > dev)
{
    if (dev == NULL)
        RW_THROW ("Device is NULL!");
    return dev->getQ (getState ());
}
void rws::swig::setQ (rw::core::Ptr< rwlibs::swig::Device > dev, rw::math::Q q)
{
    if (dev == NULL)
        RW_THROW ("Device is NULL!");
    State state = getState ();
    dev->setQ (q, state);
    setState (state);
}

void rws::swig::setTransform (rwlibs::swig::Frame* mframe, rw::math::Transform3D< double > wTframe)
{
    if (FixedFrame* ff = dynamic_cast< FixedFrame* > (mframe)) {
        ff->setTransform (wTframe);
    }
    else if (MovableFrame* mf = dynamic_cast< MovableFrame* > (mframe)) {
        State state = getState ();
        mf->setTransform (wTframe, state);
        setState (state);
    }
}

rw::math::Transform3D< double > rws::swig::wTf (rwlibs::swig::Frame* frame)
{
    return rw::kinematics::Kinematics::worldTframe (frame, getState ());
}
rw::math::Transform3D< double > rws::swig::fTf (rwlibs::swig::Frame* frame, rwlibs::swig::Frame* to)
{
    return rw::kinematics::Kinematics::frameTframe (frame, to, getState ());
}
rw::math::Transform3D< double > rws::swig::wTf (const std::string& name)
{
    return rw::kinematics::Kinematics::worldTframe (findFrame (name), getState ());
}
rw::math::Transform3D< double > rws::swig::fTf (const std::string& frame, const std::string& to)
{
    return rw::kinematics::Kinematics::frameTframe (findFrame (frame), findFrame (to), getState ());
}

void rws::swig::addGeometry (const std::string& objName, rw::geometry::Geometry::Ptr geom)
{
    std::cout << "Not Implemented" << std::endl;
}

void rws::swig::addObject (const std::string& baseFrameName, rw::geometry::Geometry::Ptr geom)
{
    std::cout << "Not Implemented" << std::endl;
}

void rws::swig::removeObject (const std::string& objName)
{
    std::cout << "Not Implemented" << std::endl;
}
