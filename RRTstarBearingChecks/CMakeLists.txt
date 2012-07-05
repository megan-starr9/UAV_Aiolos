cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#add_jar_dir(/usr/share/java)
#add_runtime_classpath(${PROJECT_SOURCE_DIR}/bin)
#add_java_source_dir(${PROJECT_SOURCE_DIR}/src/JAVA)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
#rosbuild_add_library(
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#C++ executables
rosbuild_add_executable(simulator src/simulator.cpp src/AU_UAV_ROS/SimulatedPlane.cpp src/AU_UAV_ROS/standardDefs.cpp)
rosbuild_add_executable(collisionAvoidance src/RRTstar.cpp src/AU_UAV_ROS/standardDefs.cpp 
#Algorithm Includes
src/RRTstar_lib/RRTstar.cpp
src/RRTstar_lib/Point.cpp
src/RRTstar_lib/UAV.cpp
src/RRTstar_lib/Collision.cpp
#OMPL Includes
src/RRTstar_lib/ompl/base/src/DiscreteMotionValidator.cpp
src/RRTstar_lib/ompl/base/src/GenericParam.cpp
src/RRTstar_lib/ompl/base/src/Goal.cpp
src/RRTstar_lib/ompl/base/src/GoalLazySamples.cpp
src/RRTstar_lib/ompl/base/src/GoalRegion.cpp
src/RRTstar_lib/ompl/base/src/GoalState.cpp
src/RRTstar_lib/ompl/base/src/GoalStates.cpp
src/RRTstar_lib/ompl/base/src/Planner.cpp
src/RRTstar_lib/ompl/base/src/PlannerData.cpp
src/RRTstar_lib/ompl/base/src/PlannerTerminationCondition.cpp
src/RRTstar_lib/ompl/base/src/PrecomputedStateSampler.cpp
src/RRTstar_lib/ompl/base/src/ProblemDefinition.cpp
src/RRTstar_lib/ompl/base/src/ProjectionEvaluator.cpp
src/RRTstar_lib/ompl/base/src/SpaceInformation.cpp
src/RRTstar_lib/ompl/base/src/StateSampler.cpp
src/RRTstar_lib/ompl/base/src/StateSpace.cpp
src/RRTstar_lib/ompl/base/src/StateStorage.cpp
src/RRTstar_lib/ompl/base/src/ValidStateSampler.cpp
src/RRTstar_lib/ompl/base/spaces/src/DiscreteStateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/DubinsStateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/RealVectorBounds.cpp
src/RRTstar_lib/ompl/base/spaces/src/RealVectorStateProjections.cpp
src/RRTstar_lib/ompl/base/spaces/src/RealVectorStateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/ReedsSheppStateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/SE2StateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/SE3StateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/SO2StateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/SO3StateSpace.cpp
src/RRTstar_lib/ompl/base/spaces/src/TimeStateSpace.cpp
src/RRTstar_lib/ompl/base/samplers/src/GaussianValidStateSampler.cpp
src/RRTstar_lib/ompl/base/samplers/src/MaximizeClearanceValidStateSampler.cpp
src/RRTstar_lib/ompl/base/samplers/src/ObstacleBasedValidStateSampler.cpp
src/RRTstar_lib/ompl/base/samplers/src/UniformValidStateSampler.cpp
src/RRTstar_lib/ompl/contrib/rrt_star/src/BallTreeRRTstar.cpp
src/RRTstar_lib/ompl/contrib/rrt_star/src/RRTstar.cpp
src/RRTstar_lib/ompl/control/src/ControlSampler.cpp
src/RRTstar_lib/ompl/control/src/ControlSpace.cpp
src/RRTstar_lib/ompl/control/src/PathControl.cpp
src/RRTstar_lib/ompl/control/src/PlannerData.cpp
src/RRTstar_lib/ompl/control/src/SimpleDirectedControlSampler.cpp
src/RRTstar_lib/ompl/control/src/SimpleSetup.cpp
src/RRTstar_lib/ompl/control/src/SpaceInformation.cpp
src/RRTstar_lib/ompl/control/spaces/src/DiscreteControlSpace.cpp
src/RRTstar_lib/ompl/control/spaces/src/RealVectorControlSpace.cpp
src/RRTstar_lib/ompl/control/planners/rrt/src/RRT.cpp
src/RRTstar_lib/ompl/control/planners/kpiece/src/KPIECE1.cpp
src/RRTstar_lib/ompl/control/planners/est/src/EST.cpp
src/RRTstar_lib/ompl/control/planners/syclop/src/GridDecomposition.cpp
src/RRTstar_lib/ompl/control/planners/syclop/src/Syclop.cpp
src/RRTstar_lib/ompl/control/planners/syclop/src/SyclopEST.cpp
src/RRTstar_lib/ompl/control/planners/syclop/src/SyclopRRT.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDEControlSpace.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDEEnvironment.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDESimpleSetup.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDEStatePropagator.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDEStateSpace.cpp
#src/RRTstar_lib/ompl/extensions/opende/src/OpenDEStateValidityChecker.cpp
src/RRTstar_lib/ompl/geometric/src/PathGeometric.cpp
src/RRTstar_lib/ompl/geometric/src/PathHybridization.cpp
src/RRTstar_lib/ompl/geometric/src/PathSimplifier.cpp
src/RRTstar_lib/ompl/geometric/src/SimpleSetup.cpp
src/RRTstar_lib/ompl/geometric/planners/est/src/EST.cpp
src/RRTstar_lib/ompl/geometric/planners/kpiece/src/BKPIECE1.cpp
src/RRTstar_lib/ompl/geometric/planners/kpiece/src/KPIECE1.cpp
src/RRTstar_lib/ompl/geometric/planners/kpiece/src/LBKPIECE1.cpp
src/RRTstar_lib/ompl/geometric/planners/prm/src/PRM.cpp
src/RRTstar_lib/ompl/geometric/planners/rrt/src/LazyRRT.cpp
src/RRTstar_lib/ompl/geometric/planners/rrt/src/pRRT.cpp
src/RRTstar_lib/ompl/geometric/planners/rrt/src/RRT.cpp
src/RRTstar_lib/ompl/geometric/planners/rrt/src/RRTConnect.cpp
src/RRTstar_lib/ompl/geometric/planners/sbl/src/pSBL.cpp
src/RRTstar_lib/ompl/geometric/planners/sbl/src/SBL.cpp
src/RRTstar_lib/ompl/geometric/ik/src/GAIK.cpp
src/RRTstar_lib/ompl/geometric/ik/src/HCIK.cpp
src/RRTstar_lib/ompl/tools/benchmark/src/Benchmark.cpp
src/RRTstar_lib/ompl/tools/benchmark/src/MachineSpecs.cpp
src/RRTstar_lib/ompl/tools/config/src/SelfConfig.cpp
src/RRTstar_lib/ompl/tools/debug/src/Profiler.cpp
src/RRTstar_lib/ompl/tools/multiplan/src/OptimizePlan.cpp
src/RRTstar_lib/ompl/tools/multiplan/src/ParallelPlan.cpp
src/RRTstar_lib/ompl/util/src/Console.cpp
src/RRTstar_lib/ompl/util/src/RandomNumbers.cpp
#End Includes
)
rosbuild_add_executable(coordinator src/coordinator.cpp src/AU_UAV_ROS/PlaneCoordinator.cpp src/AU_UAV_ROS/standardDefs.cpp)
rosbuild_add_executable(ControlMenu src/ControlMenu.cpp src/AU_UAV_ROS/standardDefs.cpp)
rosbuild_add_executable(KMLCreator src/KMLCreator.cpp)
rosbuild_add_executable(visualization src/visualization.cpp)
rosbuild_add_executable(evaluator src/evaluator.cpp src/AU_UAV_ROS/standardDefs.cpp)
rosbuild_add_executable(rvizTranslator src/rvizTranslator.cpp)
#rosbuild_add_executable(ompl src/RRTstar_lib/RRTstarexec.cpp)

rosbuild_link_boost(collisionAvoidance serialization)

#JAVA executables
#rospack_add_java_executable(XBeeIO com/GCS/xbee/XBeeGCS)
