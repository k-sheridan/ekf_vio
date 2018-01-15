cmake_minimum_required(VERSION 2.8.3)
project(invio)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(sophus REQUIRED)


include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/invio
	/usr/include/eigen3
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${sophus_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES invio
# CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   DEPENDS opencv
   DEPENDS sophus
)


add_library(vioParams include/invio/vioParams.h)
set_target_properties(vioParams PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(vioParams ${catkin_LIBRARIES})

add_library(vioFeature include/invio/Feature.cpp)
target_link_libraries(vioFeature ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} vioParams)

add_library(vioFrame include/invio/Frame.cpp)
target_link_libraries(vioFrame ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} vioParams vioFeature)

add_library(vio_tc_ekf include/invio/TightlyCoupledEKF.cpp)
target_link_libraries(vio_tc_ekf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} vioParams vioFeature)

add_library(klt_tracker include/invio/KLTTracker.cpp)
target_link_libraries(klt_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} vio_tc_ekf vioParams vioFeature vioFrame)

add_library(full_vio include/invio/VIO.h include/invio/VIO.cpp)
target_link_libraries(full_vio ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} vioParams vioFeature vioFrame vio_tc_ekf klt_tracker)

add_executable(invio_node src/invio_node.cpp)
target_link_libraries(invio_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} full_vio)

add_executable(invio_jacobian_test test/jacobian_test.cpp)
target_link_libraries(invio_jacobian_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} full_vio)

add_executable(invio_general_test test/test_general.cpp)
target_link_libraries(invio_general_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} full_vio)

add_executable(invio_klt_test test/klt_test.cpp)
target_link_libraries(invio_klt_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} full_vio)
