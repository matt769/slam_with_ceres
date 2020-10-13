cmake_minimum_required(VERSION 3.17)
project(ceres_simslam)

set(CMAKE_CXX_STANDARD 17)
add_definitions("-Wall -Wextra -O3")

find_package(Ceres REQUIRED)
include_directories(
        ${PROJECT_NAME}
        ${CERES_INCLUDE_DIRS}
)

add_executable(simslam ${PROJECT_NAME}/simslam.cpp)
target_link_libraries(simslam ${CERES_LIBRARIES})

add_executable(tests ${PROJECT_NAME}/test.cpp)
target_link_libraries(tests ${CERES_LIBRARIES})