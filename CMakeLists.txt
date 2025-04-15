cmake_minimum_required(VERSION 3.22)

project(DistanceCalculation)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)

add_executable(rectrect obb_display.cpp "SAT.h")
target_link_libraries(rectrect PRIVATE glm::glm-header-only)
target_link_libraries(rectrect PRIVATE glfw)
target_link_libraries(rectrect PRIVATE glad::glad)
