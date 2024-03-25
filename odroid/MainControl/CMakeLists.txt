cmake_minimum_required(VERSION 3.5)

set(ROBOT_TYPE HyLeg)         # The type of robot, support Go1 and A1 currently
set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION OFF)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT ON)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)               # Need move_base or not, ON or OFF 
set(PLATFORM Windows)
# set(PLATFORM Ubuntu)

SET(PROJECT_NAME MAINCONTROL)
PROJECT(${PROJECT_NAME})

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()


if(${ROBOT_TYPE} STREQUAL "HyLeg")
    add_definitions(-DROBOT_TYPE_Hy)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION)
     add_definitions(-DCOMPILE_WITH_SIMULATION)
endif()

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python3 COMPONENTS Interpreter Development NumPy)
    include_directories(${Python_INCLUDE_DIRS})
    target_link_libraries(MAINCONTROL ${Python_LIBRARIES})
endif()

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3  -pthread" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)


include_directories(
    "./build"
    "./inc"
    "./serial/include"

    if(${PLATFORM} STREQUAL "Ubuntu")
    "/usr/include"
    "/usr/local/include"
    "/usr/include/python3.8"
    endif()

    if(${PLATFORM} STREQUAL "Windows")
        "C:/Users/DZ/win-cross-compiler"
        # "C:/Users/DZ/win-cross-compiler/aarch64-none-linux-gnu/libc/usr/include"
        # "C:/Users/DZ/win-cross-compiler/gcc-arm-10.3/aarch64-none-linux-gnu/libc/usr/lib64"
    endif()
)

file(GLOB_RECURSE SRC_LIST
    "src/main.cpp"
    "src/*/*.cpp"
    "src/*/*.c"
    "src/*/*.cc"
)

file(GLOB_RECURSE SRC_LIST_TEST
    "src/test.cpp"
    "src/*/*.cpp"
    "src/*/*.c"
    "src/*/*.cc"
)


add_executable(${PROJECT_NAME} ${SRC_LIST})
add_executable(spi_test src/spi_test.cpp)
# add_executable(test ${SRC_LIST_TEST})

add_subdirectory(serial output EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} serial)
# AUX_SOURCE_DIRECTORY(src DIR_SRCS)



