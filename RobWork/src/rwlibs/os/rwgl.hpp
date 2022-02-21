#ifndef RWLIBS_OS_RWGL_HPP_
#define RWLIBS_OS_RWGL_HPP_


#include <rwlibs/opengl/rwgl.hpp>
#include <rw/core/os.hpp>
#ifdef RW_WIN32
#pragma message("#include <rwlibs/os/rwgl.hpp> is deprecated, use #include <rwlibs/opengl/rwgl.hpp> instead")
#else
#warning "#include <rwlibs/os/rwgl.hpp> is deprecated, use #include <rwlibs/opengl/rwgl.hpp> instead"
#endif

#endif