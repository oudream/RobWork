#include "PythonRunner.hpp"

#include <RobWorkStudioConfig.hpp>

#include <iostream>

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

using namespace rws::python;

struct PyLockGIL
{
    PyLockGIL () : gstate (PyGILState_Ensure ()) {}

    ~PyLockGIL () { PyGILState_Release (gstate); }

    PyLockGIL (const PyLockGIL&) = delete;
    PyLockGIL& operator= (const PyLockGIL&) = delete;

    PyGILState_STATE gstate;
};
class PyRelinquishGIL
{
  public:
    PyRelinquishGIL () : _thread_state (PyEval_SaveThread ()) {}
    ~PyRelinquishGIL () { PyEval_RestoreThread (_thread_state); }

    PyRelinquishGIL (const PyRelinquishGIL&) = delete;
    PyRelinquishGIL& operator= (const PyRelinquishGIL&) = delete;

    PyThreadState* _thread_state;
};

/**
 * @brief a Python lock for making sure only one thread can talk with python at a time
 */
class PythonLock
{
  public:
    PythonLock (PyThreadState* ts)
    {
        // swap interpretor
        _threadState = PyThreadState_Swap (ts);

        // Aquire Lock
        _gstate = PyGILState_Ensure ();
    }

    ~PythonLock ()
    {
        // Release Lock
        PyGILState_Release (_gstate);

        // Switch back to main thread
        PyThreadState_Swap (_threadState);
    }

  private:
    PyThreadState* _threadState;
    PyGILState_STATE _gstate;
};

PyThreadState* initial_thread = NULL;

//#define RWS_PY_DEBUG
#ifdef RWS_PY_DEBUG
std::ostream& operator<< (std::ostream& os, PyThreadState* ts)
{
    os << "PyThreadState {" << std::endl;
    os << "    pointer: " << size_t (ts) << std::endl;
    if (ts) {
        os << "    id: " << ts->id << std::endl;
        os << "    GIL_count: " << ts->gilstate_counter << std::endl;
        os << "    prev_id: " << size_t (ts->prev) << std::endl;
        os << "    next_id: " << size_t (ts->next) << std::endl;
    }
    os << "}";
    return os;
}
#endif 

PythonRunner::PythonRunner () : _threadState (NULL)
{
    initPython ();

    //Store Current Thread and go to this thread
    PyRelinquishGIL rel;

    //Acuire GIl Lock
    PyGILState_STATE gstate = PyGILState_Ensure ();

    //Store Current PythonThread
    PyThreadState* curThread = PyGILState_GetThisThreadState();

    // Create new Py interpretor
    _threadState = Py_NewInterpreter ();

    //Switch back to old PythonThread
    PyThreadState_Swap (curThread);

    //Release Gil Lock
    PyGILState_Release (gstate);

}

PythonRunner::~PythonRunner ()
{
    if (_threadState) {
        PyThreadState* ts = PyThreadState_Swap (_threadState);
        Py_EndInterpreter (_threadState);
        PyThreadState_Swap (ts);
    }
}

int PythonRunner::runCode (std::string code)
{
    PyRelinquishGIL rel;
    PythonLock swap (_threadState);
    int ret = PyRun_SimpleString (code.c_str ());
    return ret;
}

void PythonRunner::initPython ()
{
    // Initialize Python
    if (!Py_IsInitialized ()) {
#ifdef RWS_USE_PYTHON3
        wchar_t* program = Py_DecodeLocale ("RobWorkStudio Python Interpretor", NULL);
        if (program == NULL) {
            return;
        }
#endif
#ifdef RWS_USE_PYTHON2
        char program[] = "RobWorkStudio Python Interpretor";
#endif
        Py_SetProgramName (program);
        Py_InitializeEx (1);
#if PYTHON_VERSION_MINOR < 9 && defined(RWS_USE_PYTHON3)
        PyEval_InitThreads ();
#endif
        initial_thread = PyThreadState_Get ();
    }
}