#include "PyPlugin.hpp"

#include <RobWorkStudioConfig.hpp>
#include <rw/kinematics.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QFile>
#include <QGridLayout>
#include <QTextStream>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

#ifdef _GNU_SOURCE
#define _POSIX_C_SOURCE_OLD _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE

#define _XOPEN_SOURCE_OLD _XOPEN_SOURCE
#undef _XOPEN_SOURCE

#include <Python.h>

#undef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE _POSIX_C_SOURCE_OLD

#undef _XOPEN_SOURCE
#define _XOPEN_SOURCE _XOPEN_SOURCE_OLD
#endif
#ifndef _GNU_SOURCE
#include <Python.h>
#endif

using rws::RobWorkStudioPlugin;
using namespace rw::kinematics;

PyPlugin::PyPlugin (const QString& name, const QIcon& icon) :
    RobWorkStudioPlugin (std::string (name.toStdString () + std::to_string (_pyPlugins++)).c_str (),
                         icon),
    _isPythonInit (false)
{
    _base                = new QWidget (this);
    _pluginName          = name.toStdString () + std::to_string (_pyPlugins - 1);
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
        if (!Py_IsInitialized ()) {
#ifdef RWS_USE_PYTHON3
            wchar_t* program = Py_DecodeLocale (pluginName.c_str (), NULL);
            if (program == NULL) {
                return false;
            }
#endif
#ifdef RWS_USE_PYTHON2
            char program[pluginName.size ()];
            for (size_t i = 0; i < pluginName.size (); i++) {
                program[i] = pluginName[i];
            }
#endif

            Py_SetProgramName (program);
            Py_Initialize ();

            // Forward argv and argc
            std::vector< std::string > argv = {pluginName, _pluginName};
            const int argc                  = int(argv.size ());

#ifdef RWS_USE_PYTHON3
            wchar_t** argv_ = new wchar_t*[argc];
            for (int i = 0; i < argc; i++) {
                wchar_t* arg = Py_DecodeLocale (argv[i].c_str (), NULL);
                argv_[i]     = arg;
            }
#endif
#ifdef RWS_USE_PYTHON2
            char* argv_[argc];
            for (int i = 0; i < argc; i++) {
                argv_[i] = new char[argv.size ()];
                strcpy (argv_[i], argv[i].c_str ());
            }
#endif

            PySys_SetArgv (argc, argv_);

            // Python_RWS_plugin_init
            QString fileName (":/PyPlugin.py");
            QFile file (fileName);
            if (file.open (QIODevice::ReadOnly)) {
                QTextStream in (&file);
                QString text = in.readAll ();
                PyRun_SimpleString (text.toStdString ().c_str ());
            }
            else {
                RW_THROW ("Could not open PyPlugin.py");
            }

            for (int i = 0; i < argc; i++) {
                delete argv_[i];
            }
#ifdef RWS_USE_PYTHON3
            delete[] argv_;
            delete program;
#endif
            _isPythonInit = true;
        }
        else {
            std::string command = "rws_cpp_link.new_widget('" + pluginName + "')\n";
            PyRun_SimpleString (command.c_str ());
        }
        // Get Python
        std::ifstream scriptFile (pythonFilePath.c_str ());
        std::string code ((std::istreambuf_iterator< char > (scriptFile)),
                          std::istreambuf_iterator< char > ());

        PyRun_SimpleString (code.c_str ());
    }
    return exsist;
}

void PyPlugin::open (rw::models::WorkCell* workcell)
{
    if (_isPythonInit) {
        PyRun_SimpleString ("rws_cpp_link.openWorkCell()\n");
    }
}

void PyPlugin::close ()
{
    if (_isPythonInit) {
        PyRun_SimpleString ("rws_cpp_link.closeWorkCell()\n");
    }
}

void PyPlugin::stateChangedListener (const State& state)
{
    if (_isPythonInit) {
        PyRun_SimpleString ("rws_cpp_link.stateChanged()\n");
    }
}

size_t PyPlugin::_pyPlugins = 0;
