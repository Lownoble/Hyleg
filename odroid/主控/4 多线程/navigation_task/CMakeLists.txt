cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME navigation_task)
PROJECT(${PROJECT_NAME})
find_package(Threads)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3  -pthread" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)

include_directories( 
     "./build"
     "./inc"
     "/usr/local/include"
     
)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})

#target_link_libraries(${PROJECT_NAME} wiringPi)





