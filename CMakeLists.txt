cmake_minimum_required(VERSION 3.6)
project(toygraphSLAM)

set(CMAKE_CXX_STANDARD 11)
set(BOOST_ROOT /usr/lo)

set(SOURCE_FILES main.cpp Simulator.cpp Simulator.h Robot.cpp Robot.h)
add_executable(toygraphSLAM ${SOURCE_FILES})