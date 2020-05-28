cmake_minimum_required (VERSION 2.8.4) # This version was released 2011-02-16
project (Sqlite3)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})
MESSAGE(STATUS "Lib Path:" ${LIBRARY_OUTPUT_PATH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  -march=native -fPIC")

set (LIBNAME sqlite3)

include_directories(
  ./include/
)

add_library(${LIBNAME} sqlite3.c)

