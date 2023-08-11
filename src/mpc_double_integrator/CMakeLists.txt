cmake_minimum_required(VERSION 3.0)

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
    ../../lib/ocs2/ocs2_msgs/include
)

set(OCS2_USE_LIBS
    ocs2_core 
    ocs2_mpc 
    ocs2_oc 
    ocs2_ddp 
    ocs2_qp_solver 
    ocs2_robotic_tools)

set(LIBS 
    ./mpc_controller.cpp
    ./user_command.cpp
    ./utils.cpp
)

add_library(mylib STATIC ${LIBS})

include_directories(${Boost_INCLUDE_DIRS} ${ocs2_include})

add_executable(mpc_double_integrator ./mpc_double_integrator.cpp)

target_link_libraries(mylib drake::drake ${Boost_LIBRARIES} ${OCS2_USE_LIBS} pthread gflags)

target_link_libraries(mpc_double_integrator PRIVATE mylib)




