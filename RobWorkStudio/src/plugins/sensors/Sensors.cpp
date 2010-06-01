/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "Sensors.hpp"

#include <rws/RobWorkStudio.hpp>

#include <sstream>

#include <boost/foreach.hpp>

#include <rw/sensor.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rwlibs/drawable/RenderUtil.hpp>
#include <rwlibs/simulation/camera/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
//#include <rwlibs/simulation/SimulatedScanner1D.hpp>


#include <boost/foreach.hpp>
#include <QMessageBox>

#include <sstream>

using namespace rw::common;
using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::simulation;
using namespace rwlibs::drawable;
using namespace rws;



Sensors::Sensors()    :
    RobWorkStudioPlugin("Sensors", QIcon(":/sensors.png"))
{

    setupUi(this);

/*    QWidget *widget = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widget);
    widget->setLayout(lay);
    this->setWidget(widget);

    {
        QPushButton* button = new QPushButton("Grab Image");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabImage()));
    }
    {
        QPushButton* button = new QPushButton("Grab Scan25D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan25D()));
    }

    {
        QPushButton* button = new QPushButton("Grab Scan2D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan2D()));
    }
    {
        QPushButton* button = new QPushButton("Grab Scan1D");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(grabScan1D()));
    }


*/

    _timer = new QTimer(this);
    _timer->setSingleShot(false);
    _timer->setInterval(spnUpdateTime->value());
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateSim()));
    _timer->start();
}

Sensors::~Sensors()
{
    BOOST_FOREACH(SensorSet& set, _sensors) {
        set.view->close();
    }
    _sensors.clear();
}

void Sensors::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
        boost::bind(
            &Sensors::stateChangedListener,
            this,_1), this);
}


void Sensors::updateSim(){
    
    BOOST_FOREACH(SensorSet& set, _sensors) {
        getRobWorkStudio()->getView()->makeCurrent();
        set.view->makeCurrent();
        set.sensor->update(0.001*spnUpdateTime->value(), _state);
        set.view->update();
    }


}

void Sensors::stateChangedListener(const State& state)
{
    //std::cout<<"New State "<<std::endl;
    _state = state;

    // take image with camera
    //if(_camera)
    //  _camera->
}


void Sensors::open(WorkCell* workcell)
{
    _workcell = workcell;

/*    BOOST_FOREACH(SensorSet& set, _sensors) {
        delete set.sensor;
        delete set.view;
    }
    */
    _sensors.clear();

//    _simSensors.clear();
    /*
    //if(_camera!=NULL)
    //    delete _camera;
    if(_scanner25d!=NULL)
        delete _scanner25d;
    if(_scanner2d!=NULL)
        delete _scanner2d;
    if(_scanner1d!=NULL)
        delete _scanner1d;
        */

    State state = getRobWorkStudio()->getState();
    std::vector<Frame*> frames = Kinematics::findAllFrames(workcell->getWorldFrame(), workcell->getDefaultState());

    cmbSensors->clear();
    BOOST_FOREACH(Frame* frame, frames) {
        if (frame->getPropertyMap().has("Camera")) {
            cmbSensors->addItem(QString("%1:%2").arg("Camera").arg(frame->getName().c_str()), QVariant(frame->getName().c_str()));
         }
        if (frame->getPropertyMap().has("Scanner25D")) {
            cmbSensors->addItem(QString("%1:%2").arg("Scanner25D").arg(frame->getName().c_str()),  QVariant(frame->getName().c_str()));
        }
        if (frame->getPropertyMap().has("Scanner2D")) {
            cmbSensors->addItem(QString("%1:%2").arg("Scanner2D").arg(frame->getName().c_str()),  QVariant(frame->getName().c_str()));
        }


    }



    // first we do the camera

    // then we do the scanner25d

    // and drawables.

//    _cameraViewRender = RenderUtil::makeCameraViewRender(640, 480, 50);
//    _scanViewRender = RenderUtil::makeCameraViewRender(640, 480, 50);

//    gldrawer->addDrawableToFrame(_camera->getFrame(), new Drawable(_cameraViewRender));
//    gldrawer->addDrawableToFrame(_scanner25d->getFrame(), new Drawable(_scanViewRender));

 /*   frame = workcell->findFrame("ScanScreen");
    if(!frame) frame = world;
    _scanRender = ownedPtr( new RenderScan() );
    gldrawer->addDrawableToFrame(frame, new Drawable(_scanRender));

    frame = workcell->findFrame("Screen");
    if(!frame) frame = world;
*/

/*    // remember to initialize camera and scanners
    _camera->initialize();
    _camera->start();
    _scanner25d->open();

*/

    stateChangedListener(getRobWorkStudio()->getState());
}

void Sensors::close()
{}

void Sensors::on_btnDisplay_clicked(bool checked) {
    std::string frameName = cmbSensors->itemData(cmbSensors->currentIndex()).toString().toStdString();
    QStringList strings = cmbSensors->currentText().split(":");
    if (strings.size()<=1) {
        QMessageBox::critical(this, tr("Sensors"), tr("Unable to split string"));
    }
    std::string sensorName = strings[0].toStdString();
    std::cout<<"Frame Name"<<std::endl;
    Frame* frame = _workcell->findFrame(frameName);
    if (frame == NULL) {
        QMessageBox::critical(this, tr("Sensors"), tr("Unable to find frame %1").arg(frameName.c_str()));
        return;
    }

    WorkCellGLDrawer *gldrawer = getRobWorkStudio()->getWorkCellGLDrawer();
    SimulatedSensor* sensor = NULL;
    SensorView* view = NULL;

    if (sensorName == "Camera" && frame->getPropertyMap().has("Camera")) {
        double fovy;
        int width,height;
        std::string camId("Camera");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> width >> height;
        getRobWorkStudio()->getView()->makeCurrent();
        FrameGrabberPtr framegrabber = ownedPtr( new GLFrameGrabber(width,height,fovy,gldrawer) );
        SimulatedCamera *simcam = new SimulatedCamera("SimulatedCamera", framegrabber);
        sensor = simcam;


        simcam->attachTo(frame);
        simcam->initialize();
        simcam->start();

        view = new CameraView(simcam, NULL);
     }
    else if (sensorName == "Scanner25D" && frame->getPropertyMap().has("Scanner25D")) {
        double fovy;
        int width,height;
        std::string camId("Scanner25D");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> width >> height;

        Scan25DView* scanview = new Scan25DView();
        scanview->makeCurrent();
        FrameGrabber25DPtr framegrabber25d = ownedPtr( new GLFrameGrabber25D(width, height,fovy,gldrawer) );
        SimulatedScanner25D* simscan25 = new SimulatedScanner25D("SimulatedScanner25D", framegrabber25d);
        sensor = simscan25;
        simscan25->attachTo(frame);
        simscan25->open();

        scanview->initialize(simscan25);
        view = scanview;
        view->resize(512,512);
    }


    else if (sensorName == "Scanner2D" && frame->getPropertyMap().has("Scanner2D")) {
        double fovy;
        int cnt;
        std::string camId("Scanner2D");
        std::string camParam = frame->getPropertyMap().get<std::string>(camId);
        std::istringstream iss (camParam, std::istringstream::in);
        iss >> fovy >> cnt;

        Scan2DView* scanview = new Scan2DView();
        scanview->makeCurrent();
        FrameGrabber25DPtr framegrabber25d = ownedPtr( new GLFrameGrabber25D(1, cnt,fovy,gldrawer) );
        SimulatedScanner2D* simscan2D = new SimulatedScanner2D("SimulatedScanner2D", framegrabber25d);
        sensor = simscan2D;
        simscan2D->attachTo(frame);
        simscan2D->open();

        scanview->initialize(simscan2D);
        view = scanview;
        view->resize(512,512);
    }




    if (sensor != NULL && view != NULL) {
        _sensors.push_back(SensorSet(sensor, view));
        connect(view, SIGNAL(viewClosed(SensorView*)), this, SLOT(viewClosed(SensorView*)));
        view->show();
    } else {
        QMessageBox::critical(this, tr("Sensors"), tr("Failed to create sensor and view"));
        if (sensor != NULL)
            delete sensor;
        if (view != NULL)
            delete view;

    }

}


void Sensors::on_spnUpdateTime_valudChanged(int value) {
    _timer->setInterval(spnUpdateTime->value());
}

void Sensors::viewClosed(SensorView* view) {
    std::cout<<"View Closed"<<std::endl;
    for (std::vector<SensorSet>::iterator it = _sensors.begin(); it != _sensors.end(); ++it) {
        if ((*it).view == view) {
            _sensors.erase(it); //The smart pointers makes sure we delete the view and the sensor
            return;
        }
    }

}

//----------------------------------------------------------------------
#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Sensors, Sensors)
#endif
