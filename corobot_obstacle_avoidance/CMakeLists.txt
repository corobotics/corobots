cmake_minimum_required(VERSION 2.8.3)
project(corobot_obstacle_avoidance)
find_package(catkin
    REQUIRED COMPONENTS
    corobot_common
    geometry_msgs
    roscpp
    sensor_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
#add_library(corobot src/corobot.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

catkin_package(
    #DEPENDS corobot_common geometry_msgs roscpp sensor_msgs turtlebot_node
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp src/apf.cpp)
target_link_libraries(obstacle_avoidance corobot ${catkin_LIBRARIES})
