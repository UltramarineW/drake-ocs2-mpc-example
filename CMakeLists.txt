cmake_minimum_required(VERSION 3.0)
project(my_drake_test)

include(CTest)
enable_testing()

set(CMAKE_BULID_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)
add_subdirectory(lib/ocs2/)


