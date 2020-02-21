%module sdurws

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwslibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

using namespace rwlibs::swig;
using namespace rws::swig;

using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;

%}

%import <rwlibs/swig/sdurw.i> 

%pragma(java) jniclassclassmodifiers="class"

%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
%}

/********************************************
 * General utility functions
 ********************************************/

/**
 * @brief Launch an instance of RobWorkStudio
 * @return pointer to robworkstudio
 */ 
rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance();

/**
 * @brief Launch an instance of RobWorkStudio
 * @param args [in] string literal of input arguments for robworkstudio
 * @return pointer to robworkstudio
 */
rw::common::Ptr<RobWorkStudio> getRobWorkStudioInstance(const std::string& args);

/**
 * @brief incase RobWorkStudio has been launched by other means then getRobWorkStudioInstance()
 * use this function to get acces to all the build in functions
 * @param rwstudio [in] a pointer to a robworkStudio Instance
 */
void setRobWorkStudio (RobWorkStudio* rwstudio);

/**
 * @brief get a pointer to the current associated robworkstudio instance
 * @return a pointer to the current robworkStudio Instance
 */
RobWorkStudio* getRobWorkStudio();

/**
 * @brief Find out if robworkstudio is running. NOTICE only if robworkstudio started with getRobWorkStudioInstance
 * @return true if running
 */
bool isRunning();

/**
 * @brief this is used to connect to an already running instance of robworkStudio.
 * Notice. The main purpose for this function is to allow rws python plugins to find robworkstudio
 * @return The running robworkstudio instance
 */
RobWorkStudio* getRobWorkStudioFromQt ();



const State& getState ();
void setState (State& state);
rw::common::Ptr< Device > findDevice (const std::string& name);
rw::common::Ptr< JointDevice > findJointDevice (const std::string& name);
rw::common::Ptr< SerialDevice > findSerialDevice (const std::string& name);
rw::common::Ptr< TreeDevice > findTreeDevice (const std::string& name);
rw::common::Ptr< ParallelDevice > findParallelDevice (const std::string& name);
Frame* findFrame (const std::string& name);
MovableFrame* findMovableFrame (const std::string& name);
FixedFrame* findFixedFrame (const std::string& name);

void moveTo (MovableFrame* mframe, rw::math::Transform3D< double > wTframe);
void moveTo (Frame* frame, MovableFrame* mframe, rw::math::Transform3D< double > wTtcp);
void moveTo (const std::string& fname, const std::string& mname,
             rw::math::Transform3D< double > wTframe);

rw::math::Q getQ (rw::common::Ptr< Device > dev);
void setQ (rw::common::Ptr< Device > dev, rw::math::Q);

void setTransform (Frame* mframe, rw::math::Transform3D< double > wTframe);

rw::math::Transform3D< double > wTf (Frame* frame);
rw::math::Transform3D< double > fTf (Frame* frame, Frame* frame);
rw::math::Transform3D< double > wTf (const std::string& frame);
rw::math::Transform3D< double > fTf (const std::string& frame, const std::string& frame);


/********************************************
 * Qt
 ********************************************/

%nodefaultctor QString;
class QString
{
}; 

%nodefaultctor QWidget;
class QWidget 
{
};

/********************************************
 * RWS
 ********************************************/

class RWStudioView3D { 
public:
    RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent);
    void showPivotPoint(bool visible);  
    //void setDrawType(rw::graphics::DrawableNode::DrawType drawType); 
    Frame* pickFrame(int x, int y);
    rw::common::Ptr<DrawableNode> pick(int x, int y);
 
    rw::common::Ptr<WorkCellScene> getWorkCellScene(); 
    rw::common::Ptr<SceneViewer> getSceneViewer(); 
    void saveBufferToFile(const QString& filename);

};

%template (RWStudioView3DPtr) rw::common::Ptr<RWStudioView3D>;

class RobWorkStudio { 
public:
    RobWorkStudio(const PropertyMap& map);

    void openFile(const std::string& filename);

    PropertyMap& getPropertyMap();


    rw::common::Ptr<WorkCell> getWorkCell();

    rw::common::Ptr<CollisionDetector> getCollisionDetector();

    rw::common::Ptr<WorkCellScene> getWorkCellScene();

    rw::common::Ptr<RWStudioView3D> getView();

    const Path<Timed<State> >& getTimedStatePath();

    rw::common::Log& log();

    //void updateAndRepaint();
    //void setState(const State& state);
    //void setTimedStatePath(const PathTimedState& path);
    void postState(const State& state);
    void postUpdateAndRepaint();
    void postSaveViewGL(const std::string& str);
    void postTimedStatePath(const Path<Timed<State> >& path);
    void postWorkCell(rw::common::Ptr<WorkCell> workcell);
    void postOpenWorkCell(const std::string& str);
    void postExit();

    const State& getState();


    %extend {
        void setTimedStatePath(rw::common::Ptr<Path<Timed<State> > > path){
            $self->postTimedStatePath(*path);
        }

        void setState(const State& state){
            $self->postState(state);
        }

        void setWorkCell(rw::common::Ptr<WorkCell> workcell){
            $self->postWorkCell(workcell);
        }

        void openWorkCell(const std::string& file){
            $self->postOpenWorkCell(file);
        }

        void saveViewGL(const std::string& filename){
            $self->postSaveViewGL( filename );
        }

        rw::math::Transform3D<double> getViewTransform(){
            return $self->getView()->getSceneViewer()->getTransform();
        }

        void setViewTransform(rw::math::Transform3D<double> t3d){
            $self->getView()->getSceneViewer()->setTransform(t3d);
            $self->postUpdateAndRepaint();
        }

        void updateAndRepaint(){
            $self->postUpdateAndRepaint();
        }

        void fireGenericEvent(const std::string& str){
        	RW_WARN("fireGenericEvent(const std::string& str) IS DEPRECATED. PLEASE USE send and wait. These use the GenericAnyEvent");
        	
            $self->postGenericEvent(str);
        }

        void send(const std::string& id){
        	boost::any data;
        	std::cout << "Sending event: " << id << std::endl;
            $self->postGenericAnyEvent(id, data);
        }
        void send(const std::string& id, const std::string& val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, double val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, rw::math::Q val){
            $self->postGenericAnyEvent(id, val);
        }

        void send(const std::string& id, const PropertyMap& val){
            $self->postGenericAnyEvent(id, val);
        }

        int wait(const std::string& id){
            try {
            	
                $self->waitForAnyEvent(id);
            } catch ( ... ){
                return 0;
            }
            return 1;
        }

        int wait(const std::string& id, double timeout){
            try {
                $self->waitForAnyEvent(id, timeout);
            } catch ( ... ){ return 0; }
            return 1;
        }

        int wait(const std::string& id, rw::math::Q& result, double timeout=-1.0){
            try {
                boost::any data = $self->waitForAnyEvent(id, timeout);
                Q* q = boost::any_cast<Q>(&data);
                if(q!=NULL)
                    result = *q;
            } catch ( ... ){ return 0;}
            return 1;
        }

    }
    // events
    //StateChangedEvent& stateChangedEvent();
    //FrameSelectedEvent& frameSelectedEvent();
    //GenericEvent& genericEvent();
    //KeyEvent& keyEvent();
    //MousePressedEvent& mousePressedEvent();
    //StateTrajectoryChangedEvent& stateTrajectoryChangedEvent();
    //PositionSelectedEvent& positionSelectedEvent();

};

%template (RobWorkStudioPtr) rw::common::Ptr<RobWorkStudio>;

/********************************************
 * RWSLIBS GTASK
 ********************************************/
 
/********************************************
 * RWSLIBS JOG
 ********************************************/
 
/********************************************
 * RWSLIBS LOG
 ********************************************/

/********************************************
 * RWSLIBS LUA
 ********************************************/

/********************************************
 * RWSLIBS LUAEDITOR
 ********************************************/

/********************************************
 * RWSLIBS PLANNING
 ********************************************/

/********************************************
 * RWSLIBS PLAYBACK
 ********************************************/

/********************************************
 * RWSLIBS PROPERTYVIEW
 ********************************************/

/********************************************
 * RWSLIBS RWSTUDIOAPP
 ********************************************/
namespace rws{

    /**
     * @brief a RobWorkStudio main application which may be instantiated in its own thread.
     */
    class RobWorkStudioApp
     {
     public:
        /**
         * constructor
         * @param args [in] command line arguments for RobWorkStudio
         */
        RobWorkStudioApp(const std::string& args);

        //! destructor
        ~RobWorkStudioApp();

        /**
         * @brief start RobWorkStudio in its own thread
         */
        void start();
        
        /**
         * @brief start RobWorkStudio in this thread. Notice this method call will
         * block until RobWorkStudio is exited.
         * @return zero if exited normally.
         */
        int run();

        /**
         * @brief check if RobwWrkStudio is running
         * @return true if running false otherwise
         */
        bool isRunning();

        /**
         * @brief get handle to the running RobWorkStudio instance.
         * @note do not directly change Qt visualization objects, this will
         * produce segfaults. Instead use Qt events and the post* handles on
         * RobWorkStudio interface.
         * @return handle to RobWorkStudio
         */
		RobWorkStudio * getRobWorkStudio();
	
     };
}

/********************************************
 * RWSLIBS SENSORS
 ********************************************/

/********************************************
 * RWSLIBS SWIG
 ********************************************/
 
/********************************************
 * RWSLIBS TREEVIEW
 ********************************************/
