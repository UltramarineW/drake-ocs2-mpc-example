cmake_minimum_required(VERSION 3.0)

set(CARTPOLE_LIBS 
    cartpole_user_command.cpp
    cartpole_mpc_controller.cpp
)

add_library(cartpole_lib STATIC ${CARTPOLE_LIBS})

include_directories(${Boost_INCLUDE_DIRS} ${ocs2_include})

add_executable(mpc_cartpole mpc_cartpole.cpp)

target_link_libraries(cartpole_lib drake::drake ${Boost_LIBRARIES} ${OCS2_USE_LIBS} pthread gflags common)

target_link_libraries(mpc_cartpole PRIVATE cartpole_lib)





