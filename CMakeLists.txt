cmake_minimum_required(VERSION 2.8)
project(CV_Project)

#set cmake build type
IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Debug)
#   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

#print build type
MESSAGE("Build  type: " ${CMAKE_BUILD_TYPE})

#set debug compile setting
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -g  -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -g -march=native")

#release compile setting
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
 

# osx settting 
if(CMAKE_HOST_APPLE)
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG "-Wl,-rpath,")
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG_SEP ":")
   cmake_policy(SET CMP0042 NEW)
endif()



#Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV 3.0 not found.")
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
else()
   message("opencv path :" ${OpenCV_LIBS})
endif()

find_package(Eigen3 3.1.0 REQUIRED)
#find_package(Pangolin REQUIRED)

#set include file search path
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/inc
${PROJECT_SOURCE_DIR}/inc/base
${PROJECT_SOURCE_DIR}/inc/feature
${EIGEN3_INCLUDE_DIR}
#${Pangolin_INCLUDE_DIRS}
)

#set lib link path 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#build library 

set(Positioning "position")

add_library(${Positioning} SHARED
src/base/P_Config.cpp
src/base/P_Frame.cpp
src/base/P_Data.cpp
src/base/P_Writer.cpp
src/base/P_Viewer.cpp
src/base/P_Detector.cpp
src/base/P_Checker.cpp
src/base/P_CoorTrans.cpp
src/base/P_Map.cpp
src/base/P_MapPoint.cpp
src/base/P_Converter.cpp
src/base/P_Optimizer.cpp
src/base/P_PoseEstimation.cpp
src/base/P_Positioning.cpp
src/base/P_Controller.cpp
src/feature/P_Feature.cpp
src/feature/P_ORBFeature.cpp
src/feature/P_FeatureMatcher.cpp
#src/P_Factory.cpp
)

#link libs
target_link_libraries(${Positioning}
${OpenCV_LIBS}
${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
)

if(CMAKE_HOST_APPLE)
   if(CMAKE_BUILD_TYPE EQUAL Release)
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o.dylib)
   else()
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2od.dylib)
   endif()
else()
   if(CMAKE_BUILD_TYPE EQUAL Release)
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o.so)
   else()
      target_link_libraries(${Positioning}
         ${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2od.so)
   endif()
endif()

#set program running path 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output)

#build exe 
add_executable(Positioning
main.cpp)
target_link_libraries(Positioning ${Positioning})
