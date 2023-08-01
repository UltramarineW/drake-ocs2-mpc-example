cmake_minimum_required(VERSION 3.0)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/home/wujiayang/drake")

find_package(drake CONFIG REQUIRED)

set(SRCS
    ./drake_per_step_update_test.cpp
    )

add_executable(drake_system_events_test ${SRCS})
target_link_libraries(drake_system_events_test pthread gflags drake::drake)
