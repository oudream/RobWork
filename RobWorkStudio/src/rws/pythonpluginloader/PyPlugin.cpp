#include "PyPlugin.hpp"

#include <RobWorkStudioConfig.hpp>
#include <rw/kinematics.hpp>
#include <rws/RobWorkStudio.hpp>


#include <QGridLayout>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

#ifdef _GNU_SOURCE
    #define _POSIX_C_SOURCE_OLD _POSIX_C_SOURCE
    #undef _POSIX_C_SOURCE

    #define _XOPEN_SOURCE_OLD _XOPEN_SOURCE
    #undef  _XOPEN_SOURCE

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

/*namespace {
    class foo {
      public:
        foo() {
            value="test";
        }
        std::string* getValue() {
            return &value;
        }
        std::string value;
    };

    foo* Item = new foo();

    static PyObject * getRWS(PyObject *self, PyObject *args)
    {
        if (!PyArg_ParseTuple(args, ":getRWS"))
            return NULL;

        return Py_BuildValue("&O",Item);
    }
    // wrap the methods to be exposed to python in a module
    // i.e. this is a list of method descriptions for the module
    static PyMethodDef SpamMethods[] = {

        // turkey.spam_system()
        {"getRWS",    //function name
        getRWS,        //function pointer
        METH_VARARGS,       //How to parse arguments
        "do getRWS .. return RWS."}, // hint

        {NULL, NULL, 0, NULL} // sentinel.
    };

    static struct PyModuleDef spammodule = {
        PyModuleDef_HEAD_INIT,
        "spam",   // name of module 
        NULL, // module documentation, may be NULL 
        -1,       // size of per-interpreter state of the module,
                   // or -1 if the module keeps state in global variables. 
        SpamMethods
    };

    PyMODINIT_FUNC PyInit_spam(void)
    {
        return PyModule_Create(&spammodule);
    }

}*/

PyPlugin::PyPlugin () :
    RobWorkStudioPlugin (
        std::string ("PyPlugin" + std::to_string (_pyPlugins++)).c_str (),
        QIcon (":/PythonIcon.png"))
{
    _base = new QWidget (this);
    _pluginName = "PyPlugin" + std::to_string (_pyPlugins -1);
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
        #ifdef RWS_USE_PYTHON3
            wchar_t* program = Py_DecodeLocale (pluginName.c_str (), NULL);
            if (program == NULL) {
                return false;
            }
        #endif
        #ifdef RWS_USE_PYTHON2
            char program[pluginName.size()];
            for (size_t i = 0; i < pluginName.size(); i++ ) {
                program[i] = pluginName[i];
            }
        #endif
        
        Py_SetProgramName (program);
        //PyImport_AppendInittab( "spam", PyInit_spam);
        Py_Initialize ();

        // Forward argv and argc
        std::vector< std::string > argv = {pluginName, _pluginName};
        const int argc                        = int(argv.size ());

        #ifdef RWS_USE_PYTHON3
            wchar_t** argv_ = new wchar_t*[argc];
            for (int i = 0; i < argc; i++) {
                wchar_t* arg = Py_DecodeLocale (argv[i].c_str (), NULL);
                argv_[i] = arg;
            }
        #endif 
        #ifdef RWS_USE_PYTHON2
            char* argv_[argc];
            for (int i = 0; i < argc; i++) {
                argv_[i]     = new char[argv.size()];
                strcpy(argv_[i],argv[i].c_str());
            }
        #endif

        PySys_SetArgv (argc, argv_);
        // Get Python
        std::ifstream scriptFile (pythonFilePath.c_str ());
        std::string code ((std::istreambuf_iterator< char > (scriptFile)),
                          std::istreambuf_iterator< char > ());

        /*PyObject * mainModule = PyImport_AddModule("__main__");
        PyObject * hashlibModule = PyImport_ImportModule("spam");
        PyModule_AddObject(mainModule, "spam", hashlibModule);
        PyObject * module = PyModule_Create(&spammodule);*/

        PyRun_SimpleString (code.c_str ());

        for(int i = 0; i < argc ; i++) {
            delete argv_[i];
        }
        delete [] argv_;
        
        #ifdef RWS_USE_PYTHON3
            delete program;
        #endif
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
