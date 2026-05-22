cmake_minimum_required(VERSION 3.16)

add_library(lobshot OBJECT
    lobshot.cpp
)

target_link_libraries(lobshot io)
