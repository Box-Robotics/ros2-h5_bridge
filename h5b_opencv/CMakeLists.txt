cmake_minimum_required(VERSION 3.5)
project(h5b_opencv)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(h5_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

#############
##  Build  ##
#############

#-----------------------
# libh5b_opencv.so
#-----------------------

add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib/opencv.cpp
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
   )

target_compile_options(${PROJECT_NAME}
  #PRIVATE -Wall -Wextra
  PRIVATE -Wall
  )

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  )

ament_target_dependencies(${PROJECT_NAME}
  h5_bridge
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
      ${OpenCV_INCLUDE_DIRS}
    )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )
  ament_target_dependencies(${PROJECT_NAME}_test
    h5_bridge
    )
endif(BUILD_TESTING)

##############
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_package()
