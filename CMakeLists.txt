cmake_minimum_required(VERSION 3.22)

project(DistanceCalculation)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_C_COMPILER clang)
#set(CMAKE_CXX_COMPILER clang++)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# find_package(glm CONFIG REQUIRED)
# find_package(glfw3 CONFIG REQUIRED)
# find_package(glad CONFIG REQUIRED)
add_subdirectory(external/glm)
add_subdirectory(external/glfw)
add_subdirectory(external/glad)

find_package(OpenMP REQUIRED)

add_executable(rectrect obb_display.cpp SAT.h Sample.h RectRect.h Common.h GJK.h Vector.h Matrix.h Real.h)
target_link_libraries(rectrect PRIVATE glm)
target_link_libraries(rectrect PRIVATE glfw)
target_link_libraries(rectrect PRIVATE glad)
target_link_libraries(rectrect PRIVATE OpenMP::OpenMP_CXX)

target_compile_options(rectrect PRIVATE
    $<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU>>:-O3>
    $<$<AND:$<CONFIG:Release>,$<C_COMPILER_ID:GNU>>:-O3>
)

#if (MSVC)
#    target_compile_options(rectrect PRIVATE /O2 /arch:AVX2)
#endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Using GCC, enabling -mfma")
    target_compile_options(rectrect PRIVATE -O3 -mfma -march=native)
endif()

#target_compile_options(rectrect PRIVATE
#    $<$<CXX_COMPILER_ID:GNU>:-ffast-math>
#    $<$<C_COMPILER_ID:GNU>:-ffast-math>
#)
