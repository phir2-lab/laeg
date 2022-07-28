cmake_minimum_required(VERSION 2.8.3)

project(loop_exploration)

INCLUDE("Setup.cmake")

Setup()

include_directories(
    ${PROJECT_SOURCE_DIR}
)

## Specify additional locations of header files
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/src
)

file(GLOB SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${FREEGLUT_LIBRARY}
    ${ARIA_LIBRARY}
    pthread
    freeimage
)

# Build main
add_executable(exploration src/main.cpp)
target_link_libraries(exploration ${PROJECT_NAME})
