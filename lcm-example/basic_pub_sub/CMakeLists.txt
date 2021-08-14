cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

add_compile_options(-std=c++11)

set(EXTRA_LIBS lcm)

set(CMAKE_CXX_FLAGS "-O3")

# one pc one process
add_executable(basic_pub src/send_message.cpp)
target_link_libraries(basic_pub ${EXTRA_LIBS})

add_executable(basic_sub src/listener.cpp)
target_link_libraries(basic_sub ${EXTRA_LIBS})