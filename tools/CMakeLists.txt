
message(STATUS "------Build Tool Programs---------")

#build test_trace # XXX_SOURCE_DIR   XXX stand for projectname
add_executable(tool_mapdisplay
${Positioning_SOURCE_DIR}/tools/tool_mapdisplay.cpp)
target_link_libraries(tool_mapdisplay ${Positioning})
