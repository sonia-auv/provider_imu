# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
#
# \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
#
# \section LICENSE
#
# This file is part of S.O.N.I.A. software.
#
# S.O.N.I.A. software is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# S.O.N.I.A. software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(provider_imu)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(provider_imu_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    roscpp
    self_test
    diagnostic_updater
    tf
    std_msgs
    sensor_msgs
    message_generation
    sonia_common
    )

find_library(LOG4CXX_LIBRARY log4cxx)
if (NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif ()

catkin_package(
    INCLUDE_DIRS ${provider_imu_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    self_test
    diagnostic_updater
    tf
    std_msgs
    sensor_msgs
    message_runtime
    sonia_common
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE provider_imu_FILES "${provider_imu_SRC_DIR}/*.cc" "${provider_imu_SRC_DIR}/*.h")

list(REMOVE_ITEM provider_imu_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/get_id.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/${provider_imu_SRC_DIR}/${PROJECT_NAME}/reset.cc)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${provider_imu_SRC_DIR}
    ${sonia_common_INCLUDE_DIRS}
)

#==========================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${provider_imu_SRC_DIR}/${PROJECT_NAME}/main.cc ${provider_imu_FILES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${sonia_common_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node sonia_common_generate_messages_cpp)

# Utility to recover ID from IMU
add_executable(get_id ${provider_imu_SRC_DIR}/${PROJECT_NAME}/get_id.cc ${provider_imu_FILES})
target_link_libraries(get_id ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${LOG4CXX_LIBRARY})
add_dependencies(get_id sonia_common_generate_messages_cpp)

add_executable(reset ${provider_imu_SRC_DIR}/${PROJECT_NAME}/reset.cc ${provider_imu_FILES})
target_link_libraries(reset ${catkin_LIBRARIES} ${sonia_common_LIBRARIES} ${LOG4CXX_LIBRARY})
add_dependencies(reset sonia_common_generate_messages_cpp)

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
