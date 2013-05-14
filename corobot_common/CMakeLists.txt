cmake_minimum_required(VERSION 2.8.3)
project(corobot_common)
find_package(catkin
    REQUIRED COMPONENTS
    message_generation
    nav_msgs
    geometry_msgs
    roscpp
    sensor_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
    Goal.msg
    Landmark.msg
    Pose.msg
    UIConfirm.msg
    UIMessage.msg
)
add_service_files(FILES
    GetCoMap.srv
    GetLandmark.srv
    GetLandmarks.srv
    GetNeighbors.srv
    GetPixelOccupancy.srv
)
generate_messages(
    DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

catkin_package(
    DEPENDS nav_msgs geometry_msgs roscpp sensor_msgs
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
    LIBRARIES corobot
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(corobot src/corobot.cpp)
