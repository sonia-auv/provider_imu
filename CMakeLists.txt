cmake_minimum_required(VERSION 2.8.3)
project(provider_imu)

if ( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
          "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
          FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1 -O3" )

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        roscpp
        message_generation
        message_runtime
)

# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        CATKIN_DEPENDS roscpp std_msgs
)


#===============================================================================
# I N C L U D E   F I L E S

set( provider_imu_SRC_DIR "src" )

# Get all the files of the projects.
file( GLOB_RECURSE provider_imu_FILES
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

set( TARGET_NAME provider_imu_node )
add_executable( ${TARGET_NAME}
        ${provider_imu_FILES}
)



set_target_properties( ${TARGET_NAME}
        PROPERTIES
        LINKER_LANGUAGE CXX
)

#==========================================================================
# C R E A T I N G   U N I T   T E S T S



