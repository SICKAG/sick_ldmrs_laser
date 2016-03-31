cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs_driver)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure
      driver_base
      pcl_conversions
      sick_ldmrs_msgs)

find_package(Boost REQUIRED COMPONENTS system)

find_package(SickLDMRS REQUIRED)

generate_dynamic_reconfigure_options(cfg/SickLDMRSDriver.cfg)

catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure pcl_conversions sick_ldmrs_msgs
#    LIBRARIES 
    INCLUDE_DIRS include
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sick_ldmrs800001s01
    src/sick_ldmrs800001s01.cpp)
target_link_libraries(sick_ldmrs800001s01
    ${catkin_LIBRARIES}
    ${SICK_LDMRS_LIBRARIES})
add_dependencies(sick_ldmrs800001s01 ${PROJECT_NAME}_gencfg)

install(TARGETS sick_ldmrs800001s01
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
