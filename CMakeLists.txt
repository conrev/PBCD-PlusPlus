cmake_minimum_required(VERSION 3.16)
project(PBCDPP)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
option(LIBIGL_GLFW "Build target igl::glfw" ON)
include(libigl)

# Add your project files
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/pbcd/*.hpp)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

# target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)
