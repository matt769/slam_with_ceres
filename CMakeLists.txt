cmake_minimum_required(VERSION 3.17)
project(ceres_simslam)
message(WARNING ${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)
add_definitions("-Wall -Wextra -O3")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(simslam ${PROJECT_NAME}/simslam.cpp)
target_link_libraries(simslam ${CERES_LIBRARIES})
