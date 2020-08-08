project(snake_gate_detector)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

cs_add_library(${PROJECT_NAME}_lib
    src/libsnake_gate_detector.cpp
    src/snake_gate_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS})

cs_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
cs_add_executable(${PROJECT_NAME}_test src/${PROJECT_NAME}_test.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_lib)

cs_install()
cs_export()
