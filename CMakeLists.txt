cmake_minimum_required(VERSION 2.8.3)
project(dynaVINS)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport
    pcl_conversions
    pcl_ros
    roslib)

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(vins_node
    vins_estimator/rosNodeTest.cpp
    vins_estimator/estimator/parameters.cpp
    vins_estimator/estimator/estimator.cpp
    vins_estimator/estimator/feature_manager.cpp
    vins_estimator/factor/pose_local_parameterization.cpp
    vins_estimator/factor/projectionTwoFrameOneCamFactor.cpp
    vins_estimator/factor/projectionTwoFrameTwoCamFactor.cpp
    vins_estimator/factor/projectionOneFrameTwoCamFactor.cpp
    vins_estimator/factor/marginalization_factor.cpp
    vins_estimator/utility/utility.cpp
    vins_estimator/utility/visualization.cpp
    vins_estimator/utility/CameraPoseVisualization.cpp
    vins_estimator/initial/solve_5pts.cpp
    vins_estimator/initial/initial_aligment.cpp
    vins_estimator/initial/initial_sfm.cpp
    vins_estimator/initial/initial_ex_rotation.cpp
    vins_estimator/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(loop_fusion_node
    loop_fusion/pose_graph_node.cpp
    loop_fusion/pose_graph.cpp
    loop_fusion/keyframe.cpp
    loop_fusion/utility/CameraPoseVisualization.cpp
    loop_fusion/ThirdParty/DBoW/BowVector.cpp
    loop_fusion/ThirdParty/DBoW/FBrief.cpp
    loop_fusion/ThirdParty/DBoW/FORB.cpp
    loop_fusion/ThirdParty/DBoW/FeatureVector.cpp
    loop_fusion/ThirdParty/DBoW/QueryResults.cpp
    loop_fusion/ThirdParty/DBoW/ScoringObject.cpp
    loop_fusion/ThirdParty/DUtils/Random.cpp
    loop_fusion/ThirdParty/DUtils/Timestamp.cpp
    loop_fusion/ThirdParty/DVision/BRIEF.cpp
    loop_fusion/ThirdParty/VocabularyBinary.cpp
    loop_fusion/factor/ceresBA.hpp
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES}) 


