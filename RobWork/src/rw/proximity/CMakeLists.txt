set(
    FILES_CPP
    BasicFilterStrategy.cpp
    CollisionDetector.cpp
    CollisionSetup.cpp
    CollisionStrategy.cpp
    CollisionToleranceStrategy.cpp
    DistanceCalculator.cpp
    DistanceMultiStrategy.cpp
    DistanceStrategy.cpp
    ProximityCache.cpp
    ProximityCalculator.cpp
    ProximityData.cpp
    ProximityFilter.cpp
    ProximityFilterStrategy.cpp
    ProximityModel.cpp
    ProximitySetup.cpp
    ProximitySetupRule.cpp
    ProximityStrategy.cpp
    ProximityStrategyData.cpp
    Raycaster.cpp
    rwstrategy/BVTree.cpp
    rwstrategy/BVTreeCollider.cpp
    rwstrategy/BVTreeColliderFactory.cpp
    rwstrategy/BVTreeFactory.cpp
    rwstrategy/BVTreeToleranceCollider.cpp
    rwstrategy/BinaryBVTree.cpp
    rwstrategy/BinaryIdxBVTree.cpp
    rwstrategy/OBVTreeDFSCollider.cpp
    rwstrategy/ProximityStrategyRW.cpp
)

set(
    FILES_HPP
    BasicFilterStrategy.hpp
    CollisionDetector.hpp
    CollisionSetup.hpp
    CollisionStrategy.hpp
    CollisionToleranceStrategy.hpp
    DistanceCalculator.hpp
    DistanceMultiStrategy.hpp
    DistanceStrategy.hpp
    ProximityCache.hpp
    ProximityCalculator.hpp
    ProximityData.hpp
    ProximityFilter.hpp
    ProximityFilterStrategy.hpp
    ProximityModel.hpp
    ProximitySetup.hpp
    ProximitySetupRule.hpp
    ProximityStrategy.hpp
    ProximityStrategyData.hpp
    Raycaster.hpp
    rwstrategy/BVTree.hpp
    rwstrategy/BVTreeCollider.hpp
    rwstrategy/BVTreeColliderFactory.hpp
    rwstrategy/BVTreeFactory.hpp
    rwstrategy/BVTreeToleranceCollider.hpp
    rwstrategy/BinaryBVTree.hpp
    rwstrategy/BinaryIdxBVTree.hpp
    rwstrategy/OBVTreeDFSCollider.hpp
    rwstrategy/ProximityStrategyRW.hpp
)

if(RW_HAVE_PQP)
    set(FILES_CPP ${FILES_CPP} rwstrategy/TreeDistanceCalc.cpp)
    set(FILES_HPP ${FILES_HPP} rwstrategy/TreeDistanceCalc.hpp)
endif()

source_group(proximity FILES ${FILES_CPP} ${FILES_HPP})

file(GLOB SRC_FILES_CPP ${FILES_CPP})
file(GLOB SRC_FILES_HPP ${FILES_HPP})

set(RW_LIB_FILES_CPP ${RW_LIB_FILES_CPP} ${SRC_FILES_CPP} PARENT_SCOPE)
set(RW_LIB_FILES_HPP ${RW_LIB_FILES_HPP} ${SRC_FILES_HPP} PARENT_SCOPE)
