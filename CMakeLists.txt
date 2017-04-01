cmake_minimum_required(VERSION 3.6)
project(toygraphSLAM)

set(CMAKE_CXX_STANDARD 11)
find_package(Boost COMPONENTS serialization system filesystem thread program_options date_time regex timer chrono)

set(SOURCE_FILES main.cpp Simulator.cpp Simulator.h Robot.cpp Robot.h)
add_executable(toygraphSLAM ${SOURCE_FILES})

target_link_libraries(toygraphSLAM /home/kejie/CLionProjects/toygraphSLAM/gtsam-3.2.1/build/gtsam/libgtsam.so ${Boost_LIBRARIES})
