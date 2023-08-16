cmake_minimum_required(VERSION 3.0)

find_package(drake CONFIG REQUIRED)

find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

set(ocs2_include 
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_core/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_thirdparty/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_mpc/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_oc/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_ddp/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_qp_solver/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_robotic_tools/include
    ${CMAKE_SOURCE_DIR}/lib/ocs2/ocs2_msgs/include
)

set(OCS2_USE_LIBS
    ocs2_core 
    ocs2_mpc 
    ocs2_oc 
    ocs2_ddp 
    ocs2_qp_solver 
    ocs2_robotic_tools)

set(COMMON_LIBS
    common/utils.cpp
)

add_library(common ${COMMON_LIBS})

include_directories(common)

add_subdirectory(mpc_double_integrator)

add_subdirectory(mpc_cartpole)