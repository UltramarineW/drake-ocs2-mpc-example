cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_PREFIX_PATH "/home/wujiayang/drake")

find_package(drake CONFIG REQUIRED)

find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

set(ocs2_include 
    ../../lib/ocs2/ocs2_core/include
    ../../lib/ocs2/ocs2_thirdparty/include
    ../../lib/ocs2/ocs2_mpc/include
    ../../lib/ocs2/ocs2_oc/include
    ../../lib/ocs2/ocs2_ddp/include
    ../../lib/ocs2/ocs2_qp_solver/include
    ../../lib/ocs2/ocs2_robotic_tools/include
)

set(SRCS
    ./mpc_double_integrator.cpp
)

include_directories(${Boost_INCLUDE_DIRS} ${ocs2_include})

add_executable(mpc_double_integrator ${SRCS})
target_link_libraries(mpc_double_integrator pthread gflags drake::drake ${Boost_LIBRARIES} ocs2_core ocs2_mpc ocs2_oc ocs2_ddp ocs2_qp_solver ocs2_robotic_tools)

