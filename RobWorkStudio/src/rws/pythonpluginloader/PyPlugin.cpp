#include "PyPlugin.hpp"

#include <RobWorkStudioConfig.hpp>
#include <rw/kinematics.hpp>
#include <rws/RobWorkStudio.hpp>

#include <Python.h> 
#include <QGridLayout>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <streambuf>

using rws::RobWorkStudioPlugin;
using namespace rw::kinematics;

PyPlugin::PyPlugin () :
    RobWorkStudioPlugin (
        std::string ("PyPlugin" + std::to_string (_pyPlugins++)).c_str (),
        QIcon (":/PythonIcon.png"))
{
    _base = new QWidget (this);

    QGridLayout* pLayout = new QGridLayout (_base);
    _base->setLayout (pLayout);
    this->setWidget (_base);
}

PyPlugin::~PyPlugin ()
{}

bool PyPlugin::initialize (std::string pythonFilePath, std::string pluginName)
{
    bool exsist = boost::filesystem::exists (pythonFilePath);
    if (exsist) {
        getRobWorkStudio ()->stateChangedEvent ().add (
            boost::bind (&PyPlugin::stateChangedListener, this, _1), this);

        _base->setObjectName (pluginName.c_str ());

        // Initialize Python
        wchar_t* program = Py_DecodeLocale (pluginName.c_str (), NULL);
        if (program == NULL) {
            return false;
        }
        Py_SetProgramName (program);
        Py_Initialize ();

        // Forward argv and argc
        std::vector< std::string > argv = {pluginName, "somthing"};
        int argc                        = int(argv.size ());
        wchar_t* _argv[argc];
        for (int i = 0; i < argc; i++) {
            wchar_t* arg = Py_DecodeLocale (argv[i].c_str (), NULL);
            _argv[i]     = arg;
        }
        PySys_SetArgv (argc, _argv);

        // Get Python
        std::ifstream scriptFile (pythonFilePath.c_str ());
        std::string code ((std::istreambuf_iterator< char > (scriptFile)),
                          std::istreambuf_iterator< char > ());

        PyRun_SimpleString (code.c_str ());

        delete program;
    }
    return exsist;
}

void PyPlugin::stateChangedListener (const State& state)
{}

size_t PyPlugin::_pyPlugins = 0;

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN (PyPlugin);
#endif
