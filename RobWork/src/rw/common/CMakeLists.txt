set(SUBSYS_NAME sdurw_common)
set(SUBSYS_DESC
    "This is the core of the RobWork FrameWork, providing fonctinality for logging and plugins"
)
set(SUBSYS_DEPS sdurw_core)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        ConcatVectorIterator.cpp
        LogBufferedChar.cpp
        LogBufferedMsg.cpp
        LogMultiWriter.cpp
        LogFileWriter.cpp
        Timer.cpp
        TimerUtil.cpp
        VectorIterator.cpp
        Cache.cpp
        FileCache.cpp
        PairMap.cpp
        ThreadPool.cpp
        ThreadSafeQueue.cpp
        ThreadSafeStack.cpp
        ThreadSafeVariable.cpp
        ThreadTask.cpp
        Archive.cpp
        InputArchive.cpp
        OutputArchive.cpp
        INIArchive.cpp
        BINArchive.cpp
        Serializable.cpp
        ProgramOptions.cpp
    )

    set(FILES_HPP
        AnyPtr.hpp
        ConcatVectorIterator.hpp
        IOUtil.hpp
        Log.hpp
        LogWriter.hpp
        ProgramOptions.hpp
        LogBufferedChar.hpp
        LogBufferedMsg.hpp
        LogStreamWriter.hpp
        LogMultiWriter.hpp
        LogFileWriter.hpp
        Message.hpp
        Property.hpp
        PropertyBase.hpp
        PropertyMap.hpp
        PropertyType.hpp
        StringUtil.hpp
        Timer.hpp
        TimerUtil.hpp
        VectorIterator.hpp
        Cache.hpp
        FileCache.hpp
        PairMap.hpp
        Exception.hpp
        ScopedTimer.hpp
        Event.hpp
        ThreadPool.hpp
        ThreadSafeQueue.hpp
        ThreadSafeStack.hpp
        ThreadSafeVariable.hpp
        ThreadTask.hpp
        Archive.hpp
        BoostXMLParser.hpp
        DOMElem.hpp
        DOMParser.hpp
        InputArchive.hpp
        OutputArchive.hpp
        INIArchive.hpp
        BINArchive.hpp
        Serializable.hpp
        Extension.hpp
        ExtensionPoint.hpp
        ExtensionRegistry.hpp
        Plugin.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/core" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} sdurw_core)

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> ${Boost_INCLUDE_DIR}
    )

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()