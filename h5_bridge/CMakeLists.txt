cmake_minimum_required(VERSION 3.5)
project(h5_bridge)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(spdlog_vendor REQUIRED)

# XXX: This should not be needed, but the way
# spdlog is integrated in ROS2 + CMake is borked.
# I'll consider bundling spdlog with this project
# as a git submodule, but that feels wholly unnecessary.
set(SPDLOG_DIR /opt/ros/eloquent/include)

#############
##  Build  ##
#############

link_directories(
  ${HDF5_LIBRARY_DIRS}
  )

add_definitions(
  ${HDF5_DEFINITIONS}
  )

#-----------------------
# libh5_bridge.so
#-----------------------

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/err.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/h5_file.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/logging.cpp
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SPDLOG_DIR}

  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
    ${HDF5_INCLUDE_DIRS}
   )

 target_link_libraries(${PROJECT_NAME}
   PRIVATE
   ${HDF5_LIBRARIES}
  )

target_compile_options(${PROJECT_NAME}
  #PRIVATE -Wall -Wextra
  PRIVATE -Wall
  )

##############
## Install  ##
##############

install(
  TARGETS
    ${PROJECT_NAME}

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/
  DESTINATION include
  PATTERN "*~" EXCLUDE
  )

#############
##  Test   ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB TEST_SOURCES test/*.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCES})
  target_include_directories(${PROJECT_NAME}_test
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    )
endif(BUILD_TESTING)

##############
ament_export_dependencies(ament_cmake)
ament_package()
