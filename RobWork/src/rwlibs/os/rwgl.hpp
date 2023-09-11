#ifndef RWLIBS_OS_RWGL_HPP_
#define RWLIBS_OS_RWGL_HPP_

#include <rw/core/os.hpp>
#include <rwlibs/opengl/rwgl.hpp>
#ifdef RW_WIN32
#pragma message( \
    "#include <rwlibs/os/rwgl.hpp> is deprecated, use #include <rwlibs/opengl/rwgl.hpp> instead")
#else
#warning \
    "#include <rwlibs/os/rwgl.hpp> is deprecated, use #include <rwlibs/opengl/rwgl.hpp> instead"
#endif

#endif