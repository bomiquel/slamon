cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
project(slamon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg 
                  FILES GraphPoses.msg
                  TimeTracking.msg
                  TimeGraph.msg
                  TimeLoopClosing.msg
                  SubTimeLoopClosing.msg)
                  
generate_messages(DEPENDENCIES std_msgs)

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
  message(status " Using flag -std=c++20.")
elseif(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(status " Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(status " Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(status " Using flag -std=c++11.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++20, C++17, C++14 or C++11 support. SLAMON needs at least C++11 support. Please use a different C++ compiler.")
endif()

####################################################
