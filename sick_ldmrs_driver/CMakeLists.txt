cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(sick_ldmrs_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  pcl_conversions
  roscpp
  roslaunch
  sensor_msgs
  sick_ldmrs_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(SickLDMRS REQUIRED)

generate_dynamic_reconfigure_options(cfg/SickLDMRSDriver.cfg)

catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure pcl_conversions sick_ldmrs_msgs
#    LIBRARIES 
    INCLUDE_DIRS include
    )

include_directories(include ${SICK_LDMRS_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(sick_ldmrs_node
    src/sick_ldmrs_node.cpp)
target_link_libraries(sick_ldmrs_node
    ${catkin_LIBRARIES}
    ${SICK_LDMRS_LIBRARIES})
add_dependencies(sick_ldmrs_node ${PROJECT_NAME}_gencfg)
add_dependencies(sick_ldmrs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS sick_ldmrs_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
