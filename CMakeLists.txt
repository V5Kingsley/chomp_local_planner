cmake_minimum_required(VERSION 2.8.3)
project(chomp_local_planner)


set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )


find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/CHOMPPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES chomp_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)

add_library(chomp_car_trajectory src/chomp_car_trajectory.cpp)

add_library(chomp_parameters src/chomp_parameters.cpp)

add_library(chomp_obstalce_layer src/chomp_obstacle_layer.cpp)

add_library(stomp_cost src/stomp_cost.cpp)

add_library(chomp_optimizer src/chomp_optimizer.cpp)

add_library(chomp_local_planner src/chomp_planner.cpp src/chomp_planner_ros.cpp)
target_link_libraries(chomp_local_planner chomp_car_trajectory chomp_parameters chomp_optimizer stomp_cost chomp_obstalce_layer)

add_dependencies(chomp_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chomp_local_planner ${catkin_LIBRARIES})

install(TARGETS chomp_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
