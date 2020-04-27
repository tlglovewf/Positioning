cmake_minimum_required(VERSION 2.8)
project(CV_Project)

#set cmake build type
IF(NOT CMAKE_BUILD_TYPE)
#   SET(CMAKE_BUILD_TYPE Debug)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#print build type
MESSAGE("Build  type: " ${CMAKE_BUILD_TYPE})

#release compile setting
if(CMAKE_BUILD_TYPE STREQUAL "Release")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
endif()

# osx settting 
if(CMAKE_HOST_APPLE)
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG "-Wl,-rpath,")
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG_SEP ":")
   cmake_policy(SET CMP0042 NEW)
endif()

#view 
OPTION(USE_VIEW "Build with pangolin" ON)


#Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set cmake modules find path
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#ex find opencv 
find_package(OpenCV 3.0 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV 3.0 not found.")
   find_package(OpenCV 2.4.3 REQUIRED)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
else()
   message("opencv path :" ${OpenCV_LIBS})
endif()

find_package(Eigen3  REQUIRED)

if(USE_VIEW)
find_package(Pangolin REQUIRED)
   if(NOT Pangolin_FOUND)
      message(FATAL_ERROR "Pangolin not found.")
      OPTION(USE_VIEW "Build with pangolin" OFF)
   else()
      message("pangolin path : " ${Pangolin_INCLUDE_DIRS})
   endif()
endif()

#add source macro
if(USE_VIEW)
   add_definitions("-DUSE_VIEW")
endif()

FILE(GLOB_RECURSE SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp)

#set include file search path
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/inc
${PROJECT_SOURCE_DIR}/inc/base
${PROJECT_SOURCE_DIR}/inc/base/feature
${PROJECT_SOURCE_DIR}/inc/orb
${PROJECT_SOURCE_DIR}/inc/viewer
${PROJECT_SOURCE_DIR}/inc/gps
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

#set lib link path 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#build library 

set(Positioning "position")

add_library(${Positioning} SHARED ${SRC_LIST_CPP})

#link libs
target_link_libraries(${Positioning}
${OpenCV_LIBS}
${EIGEN3_LIBS}
)
if(USE_VIEW)
target_link_libraries(${Positioning}
${Pangolin_LIBRARIES}
)
endif()

if(CMAKE_HOST_APPLE)
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.dylib
         ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.dylib)
   else()
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2od.dylib
         ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2d.dylib)
   endif()
else()
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
         ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)
   else()
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2od.so
         ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2d.so)
   endif()
endif()
target_link_libraries(${Positioning}
${PROJECT_SOURCE_DIR}/Thirdparty/GeographicLib/lib/libGeographic.a)
#set program running path 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output)

#build exe 
add_executable(Positioning
main.cpp)
target_link_libraries(Positioning ${Positioning})

# add_executable(SimpleTest
# simpletest.cpp)
# target_link_libraries(SimpleTest ${Positioning})

add_executable(PoseEstimate
twoframe/poseestimate.cpp 
twoframe/ASiftDetector.cpp
)
target_link_libraries(PoseEstimate ${Positioning})

add_executable(TargetPosition  
tools/TargetPosition.cpp
tools/PosBatchHandler.cpp)
target_link_libraries(TargetPosition ${Positioning})