

set(ROS_DEPEND_PACKAGE_LIBS "")

set(geometry_msgs_DIR /opt/ros/noetic/share/geometry_msgs/cmake/)
set(roscpp_DIR /opt/ros/noetic/share/roscpp/cmake/)
set(sensor_msgs_DIR /opt/ros/noetic/share/sensor_msgs/cmake/)
set(cv_bridge_DIR /opt/ros/noetic/share/cv_bridge/cmake/)

# depend ros package
find_package(catkin REQUIRED
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
)

set(ROS_DEPEND_INCLUDE_DIRS ${DEPEND_PACKAGE_INC_DIRS} ${CMAKE_THREAD_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
set(ROS_DEPEND_PACKAGE_LIBS ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})

include_directories(${ROS_DEPEND_INCLUDE_DIRS})



