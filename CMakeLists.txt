cmake_minimum_required(VERSION 2.8.3)
project(provider_imu)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1 -O3")

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
    INCLUDE_DIRS src
    LIBRARIES 3dmgx2 ${LOG4CXX_LIBRARY}
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs self_test diagnostic_updater tf std_srvs std_msgs
)

#===============================================================================
# I N C L U D E   F I L E S

set(provider_imu_SRC_DIR "src")

# Get all the files of the projects.
file(GLOB_RECURSE provider_imu_FILES
    "${provider_imu_SRC_DIR}/*.cc"
    "${provider_imu_SRC_DIR}/*.h"
    )

# Include all the directories where source can be found.
include_directories(
    ${provider_imu_SRC_DIR}
    ${catkin_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set(TARGET_NAME provider_imu_node)

add_library(3dmgx2 src/provider_imu/3dmgx2.cc)

add_executable(${TARGET_NAME}
    src/provider_imu/imu_node.cc
    )
target_link_libraries(${TARGET_NAME} 3dmgx2 ${catkin_LIBRARIES})

set_target_properties(${TARGET_NAME}
    PROPERTIES
    LINKER_LANGUAGE CXX
    )

# Utility to recover ID from IMU
add_executable(get_id src/provider_imu/get_id.cpp)
target_link_libraries(get_id 3dmgx2 ${catkin_LIBRARIES} ${LOG4CXX_LIBRARY})

set_target_properties(get_id
    PROPERTIES
    LINKER_LANGUAGE CXX
    )

#==========================================================================
# I N S T A L L A T I O N

## Mark executables and/or libraries for installation
install(TARGETS 3dmgx2 provider_imu_node get_id
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    COPYING.lib
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

#==========================================================================
# C R E A T I N G   U N I T   T E S T S
