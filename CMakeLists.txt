cmake_minimum_required(VERSION 3.16)
project(PBCDPP)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
option(LIBIGL_GLFW "Build target igl::glfw" ON)
option(LIBIGL_COPYLEFT_TETGEN  "Build target igl_copyleft::tetgen" ON)
include(libigl)

# Add your project files
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/src/pbcd/DeformableModel.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pbcd/Solver.cpp
 #   ${CMAKE_CURRENT_SOURCE_DIR}/src/pbcd/DistanceConstraint.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw igl_copyleft::tetgen)
