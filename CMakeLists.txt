cmake_minimum_required(VERSION 3.22)

project(DistanceCalculation)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)

# find_package(glm CONFIG REQUIRED)
# find_package(glfw3 CONFIG REQUIRED)
# find_package(glad CONFIG REQUIRED)
add_subdirectory(external/glm)
add_subdirectory(external/glfw)
add_subdirectory(external/glad)

find_package(OpenMP REQUIRED)

add_executable(rectrect obb_display.cpp SAT.h Sample.h RectRect.h Common.h)
target_link_libraries(rectrect PRIVATE glm)
target_link_libraries(rectrect PRIVATE glfw)
target_link_libraries(rectrect PRIVATE glad)
target_link_libraries(rectrect PRIVATE OpenMP::OpenMP_CXX)
