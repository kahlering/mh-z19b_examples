cmake_minimum_required(VERSION 3.0)

project(mh-z19b_examples)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/mh-z19b)



option(RASPBERRY_PI_EXAMPLE "build the raspberry pi example" ON)

if(RASPBERRY_PI_EXAMPLE)
    add_executable(example_raspberry_pi ${CMAKE_CURRENT_SOURCE_DIR}/src/raspberry_pi/main.cpp)
    target_link_libraries(example_raspberry_pi mh-z19b)
endif()