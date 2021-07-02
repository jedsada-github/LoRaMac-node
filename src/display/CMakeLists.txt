## (C)2015-2021 EmOne
##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
## Authors:  Anol Paisal (EmOne)
##
project(display)
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Target
#---------------------------------------------------------------------------------------

file(GLOB ${PROJECT_NAME}_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Fonts/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/GUI/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sh1107/*.c"
)

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL ${${PROJECT_NAME}_SOURCES})

target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Fonts
    ${CMAKE_CURRENT_SOURCE_DIR}/GUI
    ${CMAKE_CURRENT_SOURCE_DIR}/sh1107
    $<TARGET_PROPERTY:board,INTERFACE_INCLUDE_DIRECTORIES>
)
