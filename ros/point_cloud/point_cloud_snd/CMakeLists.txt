
cmake_minimum_required(VERSION 3.10)


set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

project(ros_demo_point_cloud_snd)


include(${CMAKE_CURRENT_SOURCE_DIR}/ros.cmake)


set(point_cloud_snd_src
    src/point_cloud_snd.cpp
    src/ros_helper.cc
)

add_executable(${PROJECT_NAME} ${point_cloud_snd_src})

target_link_libraries(${PROJECT_NAME}
    ${ROS_DEPEND_PACKAGE_LIBS}
)

