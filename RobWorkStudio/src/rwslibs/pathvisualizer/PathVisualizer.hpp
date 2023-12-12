#ifndef RWSLIBS_PTHONEDITOR_PYEDITOR_HPP
#define RWSLIBS_PTHONEDITOR_PYEDITOR_HPP

#include "ui_PathVisualizerPlugin.h"

#include <rw/graphics/DrawableGeometryNode.hpp>
#include <rw/trajectory/Path.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <vector>


namespace rwslibs {

/**
 * @brief This plugin can be used to edit python code and run the code directly from RobWorkStudio
 */
class PathVisualizer : public rws::RobWorkStudioPlugin, private Ui::PathVisualizerPlugin
{
    Q_OBJECT
    Q_INTERFACES(rws::RobWorkStudioPlugin)
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")

  public:
    /**
     * @brief constructor
     */
    PathVisualizer();

    /**
     * @brief destructor
     */
    virtual ~PathVisualizer();

    /**
     * @copydoc RobWorkStudioPlugin::open
     */
    virtual void open(rw::models::WorkCell* workcell);

    /**
     * @copydoc RobWorkStudioPlugin::close
     */
    virtual void close();

    /**
     * @copydoc RobWorkStudioPlugin::initialize
     */
    virtual void initialize();

  private Q_SLOTS:

    void playbackChangedListener(const rw::trajectory::TimedStatePath& path);

    void on_frameSelected(const QString&);

    void on_markerSelected(const QString&);

    void on_markerSizeChanged(double);

    void on_showLines(int);

    void on_updateVisuals();

  private:
    struct DisplayData
    {
        enum { None, Ball, Arrow, Frame } marker;
        double markerSize = 0.02;
        bool showLines;

        rw::graphics::DrawableGeometryNode::Ptr markers;
        rw::graphics::DrawableGeometryNode::Ptr lines;
    };

    std::map<rw::kinematics::Frame*, DisplayData> _markers;
    rw::kinematics::Frame* _currentFrame;

    void displayMarker(rw::kinematics::Frame*);
};

}    // namespace rwslibs
#endif
