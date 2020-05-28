
message(STATUS "------Build Test Programs---------")

#build test_trace # XXX_SOURCE_DIR   XXX stand for projectname
add_executable(test_trace
${Positioning_SOURCE_DIR}/test/test_trace.cpp)
target_link_libraries(test_trace ${Positioning})

#build test_twframe
if(USE_XFEATURE)
add_executable(test_twframe
${Positioning_SOURCE_DIR}/test/test_twframe.cpp 
)
target_link_libraries(test_twframe ${Positioning})
endif()
 
add_executable(test_cpp
${Positioning_SOURCE_DIR}/test/test_cpp.cpp)
target_link_libraries(test_cpp ${Positioning})

add_executable(test_newdata
${Positioning_SOURCE_DIR}/test/test_newdata.cpp)
target_link_libraries(test_newdata ${Positioning})

add_executable(test_weiya
${Positioning_SOURCE_DIR}/test/test_weiya.cpp)
target_link_libraries(test_weiya ${Positioning})
