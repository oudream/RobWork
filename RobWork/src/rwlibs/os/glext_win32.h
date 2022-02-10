#ifndef RWLIBS_OS_GLEXT_WIN32_HPP_
#define RWLIBS_OS_GLEXT_WIN32_HPP_


#include <rwlibs/opengl/glext_win32.h>
#include <rw/core/os.hpp>
#ifdef RW_WIN32
#pragma message("#include <rwlibs/os/glext_win32.hpp> is deprecated, use #include <rwlibs/opengl/glext_win32.hpp> instead")
#else
#warning "#include <rwlibs/os/glext_win32.hpp> is deprecated, use #include <rwlibs/opengl/glext_win32.hpp> instead"
#endif

#endif
