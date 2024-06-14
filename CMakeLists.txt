cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
project(slamon)


##############
## Compiler ##
##############

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++20 or C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    add_definitions(-DCOMPILEDWITHC20)
    message(" Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(" Using flag -std=c++17.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20 or C++17 support. SLAMON needs at least C++17 support. Please use a different C++ compiler.")
endif()


#######################
## Required packages ##
#######################

find_package(
    catkin REQUIRED COMPONENTS
        tf
        roscpp
        std_msgs
        nav_msgs
        cv_bridge
        sensor_msgs
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_generation
)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)

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
    DIRECTORY 
        msg 
    FILES
        Frame.msg
        VOInfo.msg
        Keypoint.msg
)

# Declare custom ROS services
add_service_files(
    DIRECTORY
        srv
    FILES
        GetTransform.srv
        GetProjectionMatrices.srv
)

# Generate custom ROS messages & services                    
generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include
    CATKIN_DEPENDS
        message_runtime
)


###########
## Build ##
###########

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${G2O_INCLUDE_DIRS})
link_directories(${OpenCV_INCLUDE_DIRS})

# Executables
add_executable(vo_slamon 
               src/frame.cpp
               src/vo_node.cpp
               src/publisher.cpp
               src/vision_motion_modules.cpp
               src/visual_odometry.cpp)
add_executable(map_slamon
               src/map.cpp
               src/hash.cpp
               src/frame.cpp
               src/cluster.cpp
               src/map_node.cpp
               src/publisher.cpp
               src/loop_closing.cpp
               src/vision_motion_modules.cpp)
add_executable(initial_pose_server src/initial_pose_server)

# Dependencies
add_dependencies(vo_slamon ${catkin_EXPORTED_TARGETS})
add_dependencies(map_slamon ${catkin_EXPORTED_TARGETS})
add_dependencies(initial_pose_server ${catkin_EXPORTED_TARGETS})

# Linking
target_link_libraries(vo_slamon ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(map_slamon ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} cholmod ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(initial_pose_server ${catkin_LIBRARIES})