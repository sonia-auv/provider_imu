cmake_minimum_required(VERSION 2.8.3)
project(provider_imu)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1 -O3")
set(provider_imu_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp sensor_msgs
    self_test
    diagnostic_updater
    tf
    std_srvs
    std_msgs
    )

find_library(LOG4CXX_LIBRARY log4cxx)
if (NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif ()

add_service_files(
    DIRECTORY srv
    FILES AddOffset.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
    INCLUDE_DIRS ${provider_imu_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs self_test diagnostic_updater tf std_srvs std_msgs
)

#===============================================================================
# I N C L U D E   F I L E S

# Get all the files of the projects.
file(GLOB_RECURSE provider_imu_FILES "${provider_imu_SRC_DIR}/*.cc" "${provider_imu_SRC_DIR}/*.h" )

# Must remove the main files as there is serveral of them
list(REMOVE_ITEM provider_imu_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/main.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/get_id.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/imu_reset.cc)

# Include all the directories where source can be found.
include_directories(
    ${provider_imu_SRC_DIR}
    ${catkin_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${provider_imu_SRC_DIR}/${PROJECT_NAME}/main.cc ${provider_imu_FILES} )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CXX )

# Utility to recover ID from IMU
add_executable(get_id ${provider_imu_SRC_DIR}/${PROJECT_NAME}/get_id.cc ${provider_imu_FILES})
target_link_libraries(get_id ${catkin_LIBRARIES} ${LOG4CXX_LIBRARY})
set_target_properties(get_id PROPERTIES LINKER_LANGUAGE CXX )

add_executable(imu_reset ${provider_imu_SRC_DIR}/${PROJECT_NAME}/imu_reset.cc ${provider_imu_FILES})
target_link_libraries(imu_reset ${catkin_LIBRARIES} ${LOG4CXX_LIBRARY})
set_target_properties(imu_reset PROPERTIES LINKER_LANGUAGE CXX )

#==========================================================================
# I N S T A L L A T I O N

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node get_id
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}_node imu_reset
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${provider_imu_SRC_DIR}/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
