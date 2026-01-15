cmake_minimum_required(VERSION 3.0.2)
project(slamon)

# Add custom cmake modules path only for SuiteSparse if needed.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")


##############
## Compiler ##
##############

# C++17 Standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


#######################
## Required packages ##
#######################

find_package(
    catkin REQUIRED COMPONENTS
        tf2
        roscpp
        tf2_ros
        std_msgs
        nav_msgs
        cv_bridge
        sensor_msgs
        geometry_msgs
        rosgraph_msgs
        cmake_modules
        image_geometry
        image_transport
        message_filters
        tf2_geometry_msgs
        message_generation
)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIR manually
# using -DG2O_INCLUDE_DIR.
find_package(G2O REQUIRED)

# Dependencies - OpenCV:
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4") # Change path if OpenCV-4.2.0 with the contributors module is installed in another location.
find_package(OpenCV 4.2.0 REQUIRED)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - Threads:
find_package(Threads REQUIRED)

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)


###################################
## Catkin specific configuration ##
###################################

# Declare custom ROS messages
add_message_files(
    DIRECTORY msg 
    FILES Batch.msg Keyframe.msg Keypoint.msg Performance.msg
)

# Declare custom ROS services
add_service_files(
    DIRECTORY srv
    FILES GetTransform.srv GetProjectionMatrices.srv
)

# Generate custom ROS messages & services                    
generate_messages(
    DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include
    CATKIN_DEPENDS
        roscpp
        tf2_ros
        std_msgs 
        cv_bridge 
        message_runtime
        image_transport
        tf2_geometry_msgs 
)


###########
## Build ##
###########

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIR}
                    )

# Executables
add_executable(clock_extender src/clock_extender.cpp)
add_executable(initial_pose_server_node 
               src/initial_pose_server.cpp
               src/initial_pose_server_node.cpp)
add_executable(slamon_map
               src/map.cpp
               src/hash.cpp
               src/frame.cpp
               src/cluster.cpp
               src/map_node.cpp
               src/mapmaker.cpp
               src/publisher.cpp
               src/loop_closer.cpp
               src/motion_estimator.cpp
               src/keypoint_matcher.cpp
               )
add_executable(slamon_vo
               src/frame.cpp
               src/vo_node.cpp
               src/publisher.cpp
               src/visual_odometer.cpp
               src/motion_estimator.cpp
               src/keypoint_matcher.cpp
               src/keypoint_detector.cpp
               )

# Dependencies
add_dependencies(clock_extender ${catkin_EXPORTED_TARGETS})
add_dependencies(initial_pose_server_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(slamon_map ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(slamon_vo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Linking
target_link_libraries(clock_extender ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(initial_pose_server_node ${catkin_LIBRARIES})
target_link_libraries(slamon_vo 
                      ${OpenCV_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      ${EIGEN3_LIBRARIES} 
                      ${CMAKE_THREAD_LIBS_INIT}
                     )
target_link_libraries(slamon_map 
                      ${OpenCV_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      ${EIGEN3_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      ${G2O_LIBRARIES}
                      ${SUITESPARSE_LIBRARIES} 
                      cholmod
                     )


##################
## Installation ##
##################

install(TARGETS
        clock_extender
        initial_pose_server_node
        slamon_map
        slamon_vo
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Install scripts.
install(PROGRAMS 
        scripts/delayed_rosbag_play.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )