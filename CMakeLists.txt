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
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native -fPIC")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -fPIC")
endif()

# osx settting 
if(CMAKE_HOST_APPLE)
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG "-Wl,-rpath,")
   set(CMAKE_SHARED_LIBRARY_RUNTIME_C_FLAG_SEP ":")
   cmake_policy(SET CMP0042 NEW)
endif()

#view 
OPTION(USE_VIEWER            "Build with pangolin"   ON)
message("USE_VIEWER          option is ${USE_VIEWER}")

OPTION(USE_XFEATURE          "Build with sift"       ON)
message("USE_XFEATURE        option is ${USE_XFEATURE}")

OPTION(BUILD_TEST_PROGRAMS   "Build test program"    ON) 
message("BUILD_TEST_PROGRAMS option is ${BUILD_TEST_PROGRAMS}")

OPTION(BUILD_TOOL_PROGRAMS   "Build tool program"    ON)
message("BUILD_TOOL_PROGRAMS option is ${BUILD_TOOL_PROGRAMS}")


if (USE_XFEATURE)
   add_definitions(-DUSE_CVXFEATURE)
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
   message("OpenCV Found.")
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
else()
message(STATUS "No Viewer!!!")
endif()

#add source macro
if (USE_VIEWER)
   add_definitions("-DUSE_VIEW")
endif()

FILE(GLOB_RECURSE SRC_LIST_CPP ${PROJECT_SOURCE_DIR}/src/*.cpp)

# recurrence include directories
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               #check directory
      #   message("include dir: " ${root_dir})
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
${PROJECT_SOURCE_DIR}/Thirdparty/log4cpp/include/
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

#set program running path 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output)

add_executable(TargetPosition  
main/TargetPosition.cpp
main/PosBatchHandler.cpp)
target_link_libraries(TargetPosition ${Positioning})

set(CMAKE_INSTALL_PREFIX /usr/local/)


if(CMAKE_HOST_APPLE)
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      FILE(GLOB DEPEND_LIBS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.dylib ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.a)
      #remove target so name 
      LIST(REMOVE_ITEM DEPEND_LIBS   ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${Positioning}.dylib)
   else()
      FILE(GLOB DEPEND_LIBS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*d.dylib ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.a)
      #remove target so name 
      LIST(REMOVE_ITEM DEPEND_LIBS   ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${Positioning}d.dylib)
   endif()
else()
   if(CMAKE_BUILD_TYPE STREQUAL "Release")
      FILE(GLOB DEPEND_LIBS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.a)
      #remove target so name 
      LIST(REMOVE_ITEM DEPEND_LIBS   ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${Positioning}.so)
      LIST(REMOVE_ITEM DEPEND_LIBS   ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*d.so)
   else()
      FILE(GLOB DEPEND_LIBS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*d.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/*.a)
      #remove target so name 
      LIST(REMOVE_ITEM DEPEND_LIBS   ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/lib${Positioning}d.so)
   endif()
endif()

target_link_libraries(${Positioning}
${DEPEND_LIBS}
)
target_link_libraries(${Positioning} dl)
target_link_libraries(${Positioning} pthread)

#for  install
set_target_properties(TargetPosition PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(${Positioning} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS TargetPosition ${Positioning}
       RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

message("Depdents on :")
FOREACH(src ${DEPEND_LIBS})
MESSAGE( ${src} )
ENDFOREACH() 

#ARCHIVE DESTINATION
install(FILES ${DEPEND_LIBS}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(FILES ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ORBvoc.bin 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)


#uninstall
include(cmake/PositionUninstall.cmake)

#if build test program
if(BUILD_TEST_PROGRAMS)
   add_subdirectory(test)
endif()

if(BUILD_TOOL_PROGRAMS)
   add_subdirectory(tools)
endif()