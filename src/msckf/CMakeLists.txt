cmake_minimum_required(VERSION 2.8.12)
project(msckf)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Weverything -Wno-c++98-compat -Wno-c++98-compat-pedantic -Wno-deprecated-declarations -Wno-global-constructors")


# Dependencies
find_package(Eigen3)
find_package(Boost REQUIRED log log_setup)
add_definitions(-DBOOST_LOG_DYN_LINK)  # needed to log across modules
find_package(OpenCV)

# Catkin dependencies
set(catkin_deps roscpp roslib nodelet cv_bridge image_transport std_msgs geometry_msgs sensor_msgs)
find_package(catkin REQUIRED ${catkin_deps} message_generation)

# Messages
add_message_files()
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS EIGEN3
    LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

include_directories(include)

# Targets
add_library(${PROJECT_NAME} src/msckf/EKF.cpp src/msckf/estimation.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_library(feature_tracking_nodelet src/msckf/feature_tracking_nodelet.cpp)
target_link_libraries(feature_tracking_nodelet
    ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(feature_tracking_node src/msckf/feature_tracking_node.cpp)
target_link_libraries(feature_tracking_node ${catkin_LIBRARIES})

add_library(msckf_nodelet src/msckf/msckf_nodelet.cpp)
target_link_libraries(msckf_nodelet
    ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(msckf_node src/msckf/msckf_node.cpp)
target_link_libraries(msckf_node ${catkin_LIBRARIES})


# Install
install(TARGETS ${PROJECT_NAME} feature_tracking_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
    )

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
    nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

# Testing
if (CATKIN_ENABLE_TESTING)
    find_package(GTest)
    include_directories(SYSTEM ${GTEST_INCLUDE_DIR})

    catkin_add_gtest(test_${PROJECT_NAME}
        src/test/test_ekf.cpp
        src/test/test_estimation.cpp
        src/test/test_util.cpp
        src/test/test_main.cpp)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

    catkin_add_gtest(test_sim
        src/test/test_sim.cpp
        src/test/test_main.cpp)
    target_link_libraries(test_sim ${PROJECT_NAME})


endif (CATKIN_ENABLE_TESTING)

#add_executable(scratch src/test/scratch.cpp)
#add_executable(scratch2 src/test/scratch2.cpp)
