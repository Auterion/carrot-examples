cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

project(carrot-mavsdk-offboard)
find_package(MAVSDK REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS} /usr/local/include/mavsdk)

set(HEADER_FILES
    example_2_mission_execution/mission_planner.hpp
    include/carrot_interface/carrot_interface.hpp
)

add_library(carrot-lib INTERFACE
        ${HEADER_FILES}        
)

add_executable(mission_execution example_2_mission_execution/mission_execution.cpp)

target_link_libraries(mission_execution carrot-lib MAVSDK::mavsdk)

add_executable(simple_waypoints example_1_simple_waypoints/simple_waypoints.cpp)

target_link_libraries(simple_waypoints carrot-lib MAVSDK::mavsdk)