#ifndef RWHW_RWATC3DG_HPP
#define RWHW_RWATC3DG_HPP

#include <rw/common/os.hpp>

#ifdef RW_WIN32
#include <stdio.h>     // printf
#include <stdlib.h>    // exit() function
#include <string.h>    // string handling
#include <time.h>      // needed for time functions
#include <windows.h>
#endif

#ifdef RW_MAC
#define MAC
#endif

#ifdef RW_LINUX
#define LINUX
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#endif

#include <ATC3DG.h>

#endif
