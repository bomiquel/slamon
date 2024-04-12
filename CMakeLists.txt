cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
project(slamon)


##############
## Compiler ##
##############

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++20, C++17, C++14 or C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
    add_definitions(-DCOMPILEDWITHC20)
    message(" Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(" Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(" Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(" Using flag -std=c++11.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20, C++17, C++14 or C++11 support. SLAMON needs at least C++11 support. Please use a different C++ compiler.")
endif()


#######################
## Required packages ##
#######################

find_package(
    catkin REQUIRED COMPONENTS
        tf
        roscpp
        cv_bridge
        sensor_msgs
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_generation
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)


###################################
## Catkin specific configuration ##
###################################

# Declare custom ROS messages
add_message_files(
    DIRECTORY 
        msg 
    FILES
        GraphPoses.msg
        TimeTracking.msg
        TimeGraph.msg
        TimeLoopClosing.msg
        SubTimeLoopClosing.msg
)

# Declare custom ROS services
add_service_files(
    DIRECTORY
        srv
    FILES
        GetPose.srv
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

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INCLUDE_DIRS})

# Executables
add_executable(slam 
               src/node.cpp
               src/frame.cpp
               src/caster.cpp
               src/publisher.cpp
               src/vision_modules.cpp
               src/visual_odometry.cpp)
add_executable(initial_pose_server src/initial_pose_server)

# Dependencies
add_dependencies(slam ${catkin_EXPORTED_TARGETS})
add_dependencies(initial_pose_server ${catkin_EXPORTED_TARGETS})

# Linking
target_link_libraries(slam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(initial_pose_server ${catkin_LIBRARIES})