cmake_minimum_required(VERSION 3.0)

project(mh-z19b_examples)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lib/mh-z19b lib/mh-z19b)

add_executable(example_raspberry_pi ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(example_raspberry_pi mh-z19b)
