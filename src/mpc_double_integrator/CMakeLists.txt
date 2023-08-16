cmake_minimum_required(VERSION 3.0)

set(DOUBLE_INTEGRATOR_LIBS 
    double_integrator_user_command.cpp
    double_integrator_mpc_controller.cpp
)

add_library(double_integrator_lib STATIC ${DOUBLE_INTEGRATOR_LIBS})

include_directories(${Boost_INCLUDE_DIRS} ${ocs2_include})

add_executable(mpc_double_integrator mpc_double_integrator.cpp)

target_link_libraries(double_integrator_lib drake::drake ${Boost_LIBRARIES} ${OCS2_USE_LIBS} pthread gflags common)

target_link_libraries(mpc_double_integrator PRIVATE double_integrator_lib)




