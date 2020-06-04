
message(STATUS "------Build Tool Programs---------")

#Read All Tool Files And Build Test Program
FILE(GLOB TOOL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FOREACH(TOOL_FILE ${TOOL_FILES})
    STRING(REGEX REPLACE ".*/(.*)/(.*)/*.cpp" "\\2" TOOL_NAME ${TOOL_FILE})
    MESSAGE("BUILD TOOL PROGRAM " ${TOOL_NAME})
    add_executable(${TOOL_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL_NAME}.cpp)
    target_link_libraries(${TOOL_NAME} ${Positioning})
ENDFOREACH()
