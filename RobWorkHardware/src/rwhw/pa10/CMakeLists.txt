set(SUBSYS_NAME sdurwhw_pa10)
set(SUBSYS_DESC "Driver for pa10 robot")
set(SUBSYS_DEPS)

set(build TRUE)
set(DEFAULT FALSE)
set(REASON "")

if(${BUILD_pa10})
    if(DEFINED WIN32 OR DEFINED MINGW)
        set(BUILD_pa10 FALSE)
        set(REASON "Linux only component!")
    else()
        set(LINUX_HEADER_LIBRARY /usr/src/linux/include /usr/src)

        include_directories(${LINUX_HEADER_LIBRARY})
        # message("Search for PA10 dependencies")
        find_file(delay_h_file "linux/delay.h" ${LINUX_HEADER_LIBRARY})
        find_file(system_h_file "asm/system.h" ${LINUX_HEADER_LIBRARY})

        if(NOT (delay_h_file AND system_h_file))
            set(BUILD_pa10 FALSE)
            set(REASON "Missing linux headers: linux/delay.h and asm/system.h!")
        endif()

    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    set(pa10files ./PA10.cpp ./PA10Driver.cpp ./PA10Virtual.cpp ./smsclib.c)

    add_library(${SUBSYS_NAME} ${pa10files})
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
endif()
