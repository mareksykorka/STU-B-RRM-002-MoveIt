cmake_minimum_required(VERSION 3.0.2)
project(zad2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    moveit_visual_tools
    moveit_ros_planning_interface
)

catkin_package()

###########
## Build ##
###########
include_directories(
    include
    include/zad2/geometry_msgs_ext
    include/zad2/trajectory
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_model
        src/model/model_publisher_node.cpp
)
add_executable(${PROJECT_NAME}_trajectory_plan
        src/trajectory/trajectory_plan_node.cpp
        src/trajectory/plan_interface.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_model ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_trajectory_plan ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_trajectory_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})