cmake_minimum_required(VERSION 3.15)

project(glad C CXX)


    add_library(glad
        ./include/glad/glad.h 
        ./include/KHR/khrplatform.h
        ./src/glad.c
    )


target_include_directories(glad PUBLIC ./include)