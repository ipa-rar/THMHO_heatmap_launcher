cmake_minimum_required(VERSION 3.0.2)
project(thmho_heatmap_generator)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        nav_msgs
        roscpp
        tf
)

catkin_package(
    CATKIN_DEPENDS nav_msgs costmap_2d roscpp tf
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
)

add_executable(costmap_generator src/costmap_gen.cpp)
target_link_libraries(costmap_generator
  ${catkin_LIBRARIES}
)

add_executable(costmap_aggregator src/costmap_aggregator.cpp)
target_link_libraries(costmap_aggregator
  ${catkin_LIBRARIES}
)

install(TARGETS costmap_generator costmap_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS costmap_aggregator costmap_aggregator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
