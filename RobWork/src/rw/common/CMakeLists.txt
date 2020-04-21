SET(FILES_CPP
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
  ProgramOptions.cpp
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
)

SET(FILES_HPP
  AnyPtr.hpp 
  ConcatVectorIterator.hpp
  IOUtil.hpp
  Log.hpp
  LogWriter.hpp
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
  ProgramOptions.hpp
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

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
SET(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)
