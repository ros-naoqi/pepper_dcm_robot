cmake_minimum_required(VERSION 2.8.3)
project(pepper_dcm_driver)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  hardware_interface
  controller_manager
  pepper_dcm_msgs
  sensor_msgs
  urdf
  kdl_parser
)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
    qi
    qitype
)

find_package(Boost)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs tf std_msgs pepper_dcm_msgs sensor_msgs hardware_interface controller_manager)

if( NAOqi_FOUND AND Boost_FOUND)
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories( include
        ${catkin_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS}
    	${Boost_INCLUDE_DIRS})
    add_executable(pepper_dcm_driver src/pepper_driver.cpp src/pepper.cpp)
    target_link_libraries(pepper_dcm_driver
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})
    add_dependencies(pepper_dcm_driver ${catkin_EXPORTED_TARGETS})

    install(TARGETS pepper_dcm_driver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()
