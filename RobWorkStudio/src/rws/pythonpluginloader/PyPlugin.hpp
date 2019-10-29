#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rw/kinematics/State.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <string>

class PyPlugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES (rws::RobWorkStudioPlugin)
  public:
    //! @copydoc rws::RobWorkStudioPlugin::RobWorkStudioPlugin
    PyPlugin ();

    /**
     * @brief Initialize the Python plugin
     * @param pythonFilePath [in] Full path to the python file
     * @param pluginName [in] the name of the plugin to be used when locating
     * the widget in python
     *
     * @return true if succes otherwise false
     */
    bool initialize (std::string pythonFilePath, std::string pluginName);

    void open(rw::models::WorkCell* workcell);

    void close();

  protected:
    /**
     * @brief simple destructer
     */
    ~PyPlugin ();
  private Q_SLOTS:

    void stateChangedListener (const rw::kinematics::State& state);

  private:
    static size_t _pyPlugins;
    std::string _pythonFilePath;
    std::string _pluginName;
    QWidget* _base;
    bool _isPythonInit;
};

#endif /*SAMPLEPLUGIN_HPP*/
