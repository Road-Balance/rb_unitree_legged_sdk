cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include
"/usr/include/eigen3"
"/usr/local/include/eigen3")

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(stairs_climbing examples/stairs_climbing.cpp)
target_link_libraries(stairs_climbing ${EXTRA_LIBS})

add_executable(rb_test examples/rb_test.cpp)
target_link_libraries(rb_test ${EXTRA_LIBS})

add_executable(example_dance examples/example_dance.cpp)
target_link_libraries(example_dance ${EXTRA_LIBS})

add_executable(bezier_test examples/bezier_test.cpp)
target_link_libraries(bezier_test ${EXTRA_LIBS})

add_executable(Kinematics_test examples/Kinematics_test.cpp)
target_link_libraries(Kinematics_test ${EXTRA_LIBS})

add_executable(example_wirelessHandle examples/example_wirelessHandle.cpp)
target_link_libraries(example_wirelessHandle ${EXTRA_LIBS})

# one pc multi process
add_executable(lcm_server_high examples/lcm_server_high.cpp)
target_link_libraries(lcm_server_high ${EXTRA_LIBS})

add_executable(lcm_server_low examples/lcm_server_low.cpp)
target_link_libraries(lcm_server_low ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})

add_subdirectory(pybind11)
pybind11_add_module(robot_interface_high_level src/python_interface_high_cmd.cpp)
target_link_libraries(robot_interface_high_level ${EXTRA_LIBS})