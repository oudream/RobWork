#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rwslibs/pathvisualizer/PathVisualizer.hpp>

#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

using rw::kinematics::State;
using rw::models::WorkCell;
using namespace rwslibs;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::trajectory;

using rws::RobWorkStudioPlugin;

PathVisualizer::PathVisualizer() :
    RobWorkStudioPlugin("PathVisualizer", QIcon(":/PathIcon.png")), _currentFrame(NULL) {
    setupUi(this);

    connect(_cb_frameSelect,
            SIGNAL(currentTextChanged(const QString&)),
            this,
            SLOT(on_frameSelected(const QString&)));

    connect(_cb_markerSelect,
            SIGNAL(currentTextChanged(const QString&)),
            this,
            SLOT(on_markerSelected(const QString&)));

    connect(_sb_markerSize, SIGNAL(valueChanged(double)), this, SLOT(on_markerSizeChanged(double)));
    connect(_check_showLines, SIGNAL(stateChanged(int)), this, SLOT(on_showLines(int)));
    connect(_btn_update, SIGNAL(pressed()), this, SLOT(on_updateVisuals()));
}

PathVisualizer::~PathVisualizer() {}

void PathVisualizer::initialize() {
    getRobWorkStudio()->stateTrajectoryChangedEvent().add(
        boost::bind(&PathVisualizer::playbackChangedListener, this, boost::arg<1>()), this);
}

void PathVisualizer::open(WorkCell* workcell) {
    if(workcell == NULL) {
        _cb_frameSelect->clear();
        _markers.clear();
        return;
    }
    for(Frame* f : workcell->getFrames()) { _cb_frameSelect->addItem(f->getName().c_str()); }
}

void PathVisualizer::close() {
    _cb_frameSelect->clear();
    _markers.clear();
}

void PathVisualizer::playbackChangedListener(const rw::trajectory::TimedStatePath& path) {
    for(auto [key, value] : _markers) { displayMarker(key); }
}

void PathVisualizer::on_updateVisuals() {
    displayMarker(_currentFrame);
}

void PathVisualizer::on_frameSelected(const QString& frame) {
    if(getRobWorkStudio()->getWorkCell().isNull()) return;
    _currentFrame = getRobWorkStudio()->getWorkCell()->findFrame(frame.toStdString());

    _cb_markerSelect->setCurrentIndex(int(_markers[_currentFrame].marker));
    _sb_markerSize->setValue(_markers[_currentFrame].markerSize*100);
    _check_showLines->setChecked(_markers[_currentFrame].showLines);
}

void PathVisualizer::on_markerSelected(const QString& marker) {
    if(marker == "None") _markers[_currentFrame].marker = DisplayData::None;
    else if(marker == "Ball") _markers[_currentFrame].marker = DisplayData::Ball;
    else if(marker == "Arrow") _markers[_currentFrame].marker = DisplayData::Arrow;
    else if(marker == "Frame") _markers[_currentFrame].marker = DisplayData::Frame;
}

void PathVisualizer::on_markerSizeChanged(double value) {
    _markers[_currentFrame].markerSize = value / 100;
}

void PathVisualizer::on_showLines(int value) {
    _markers[_currentFrame].showLines = (value == Qt::CheckState::Checked);
}

void PathVisualizer::displayMarker(rw::kinematics::Frame* frame) {
    if(frame == NULL) return;

    const TimedStatePath& path           = getRobWorkStudio()->getTimedStatePath();
    rw::graphics::WorkCellScene::Ptr wcs = getRobWorkStudio()->getWorkCellScene();
    DisplayData& data                    = _markers[frame];
    rw::kinematics::Frame* world         = getRobWorkStudio()->getWorkCell()->getWorldFrame();

    std::string modelName = frame->getName() + "_Waypoints";

    bool update = false;
    if(!data.markers.isNull()) {
        wcs->removeDrawable(data.markers->getName());
        update       = true;
        data.markers = NULL;
    }

    if(!data.lines.isNull()) {
        wcs->removeDrawable(data.lines->getName());
        update     = true;
        data.lines = NULL;
    }

    bool showMarkers = data.marker != DisplayData::None;
    bool showLines   = data.showLines;

    if(path.empty()) {
        showMarkers = false;
        showLines   = false;
    }

    if(showMarkers) {
        update = true;

        for(const TimedState& ts : path) {
            const State& state = ts.getValue();
            Geometry::Ptr geo;

            // Create Geometry
            if(data.marker == DisplayData::Arrow) {
                geo = Geometry::makeCone(data.markerSize, 0, data.markerSize);
                geo->setColor(0.0f, 0.0f, 1.0f);
            }
            else if(data.marker == DisplayData::Frame) {
                geo = Geometry::makeSphere(data.markerSize / 10.0);
                geo->setColor(1.0f, 1.0f, 1.0f);
            }
            else {
                geo = Geometry::makeSphere(data.markerSize);
                geo->setColor(0.0f, 1.0f, 0.0f);
            }

            geo->setTransform(frame->wTf(state));

            // Create marker Drawable or make new if none exist
            if(data.markers.isNull()) {
                data.markers = wcs->addGeometry(frame->getName() + "_Waypoints", geo, world);
            }
            else { data.markers->addGeometry(geo); }

            // addFrameAxis if relevant
            if(data.marker == DisplayData::Frame) { data.markers->addFrameAxis(data.markerSize); }
        }
    }

    if(showLines) {
        update                    = true;
        StateTrajectory::Ptr traj = TrajectoryFactory::makeLinearTrajectory(path);
        double t                  = 0;
        std::vector<rw::geometry::Line> lines;

        while(t < traj->duration()) {
            rw::math::Vector3D<double> pos = frame->wTf(traj->x(t)).P();
            rw::math::Vector3D<double> pos2;

            double dt = 0.1;
            for(size_t i = 0; i < 100; i++) {
                pos2 = frame->wTf(traj->x(t + dt)).P();

                double dist = (pos2 - pos).norm2();
                if(dist > 0.0101) { dt /= 2.0; }
                else if(dist < 0.0990) { dt *= 2.2; }
                else { break; }
            }
            pos2 = frame->wTf(traj->x(t + dt)).P();
            lines.push_back(rw::geometry::Line(pos, pos2));
            t += dt;
        }
        rw::geometry::Line last = lines.back();
        lines.pop_back();

        // Create marker Drawable or make new if none exist

        data.lines = wcs->addLines(frame->getName() + "_line", lines, world);
        data.lines->addLine(last.p1(), last.p2(), 3);
    }

    if(update) getRobWorkStudio()->postUpdateAndRepaint();
}
