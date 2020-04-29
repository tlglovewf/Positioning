cmake_minimum_required(VERSION 2.8)
project(Positioning)

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
OPTION(USE_VIEWER    "Build with pangolin" OFF)
OPTION(USE_XFEATURE  "Build with sift"     OFF)
OPTION(BUILD_TESTS   "Build test program"  OFF)

if(USE_XFEATURE)
   add_definitions("-DUSE_CVXFEATURE")
endif()

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

if(USE_VIEWER)
find_package(Pangolin REQUIRED)
   if(NOT Pangolin_FOUND)
      message(FATAL_ERROR "Pangolin not found.")
      OPTION(USE_VIEWER "Build with pangolin" OFF)
   else()
      message("pangolin path : " ${Pangolin_INCLUDE_DIRS})
   endif()
endif()

#add source macro
if (USE_VIEWER)
   add_definitions("-DUSE_VIEW")
endif()

FILE(GLOB_RECURSE SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp)

# recurrence include directories
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               #check directory
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) #insert sub dirs 
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) #
        endif()
    endforeach()
endfunction()


include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)
include_sub_directories_recursively(${PROJECT_SOURCE_DIR}/inc)


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
if(USE_VIEWER)
target_link_libraries(${Positioning}
${Pangolin_LIBRARIES}
)
endif()

if(CMAKE_HOST_APPLE)
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      target_link_libraries(${Positioning}
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o.dylib
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libDBoW2.dylib)
   else()
      target_link_libraries(${Positioning}
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2od.dylib
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libDBoW2d.dylib)
   endif()
else()
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      target_link_libraries(${Positioning}
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o.so
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libDBoW2.so)
   else()
      target_link_libraries(${Positioning}
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2od.so
         ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libDBoW2d.so)
   endif()
endif()
target_link_libraries(${Positioning}
${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libGeographic.a)
#set program running path 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()

add_executable(TargetPosition  
tools/TargetPosition.cpp
tools/PosBatchHandler.cpp)
target_link_libraries(TargetPosition ${Positioning})

set(CMAKE_INSTALL_PREFIX /usr/local/)

install(TARGETS TargetPosition ${Positioning}
       RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

FILE(GLOB DEPEND_LIBS ${PROJECT_SOURCE_DIR}/lib/*.so)

#ARCHIVE DESTINATION
install(FILES ${DEPEND_LIBS}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

#uninstall
include(cmake/PositionUninstall.cmake)