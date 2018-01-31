cmake_minimum_required(VERSION 2.8.3)
project(ekf_vio)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Werror -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DEIGEN_NO_DEBUG")

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
	include/ekf_vio
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


add_library(ekfvio_Params include/ekf_vio/Params.cpp)
set_target_properties(ekfvio_Params PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(ekfvio_Params ${catkin_LIBRARIES})

add_library(ekfvio_Feature include/ekf_vio/Feature.cpp)
target_link_libraries(ekfvio_Feature ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ekfvio_Params)

add_library(ekfvio_Frame include/ekf_vio/Frame.cpp)
target_link_libraries(ekfvio_Frame ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_Params ekfvio_Feature)

add_library(vio_tc_ekf include/ekf_vio/TightlyCoupledEKF.cpp)
target_link_libraries(vio_tc_ekf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ekfvio_Params ekfvio_Feature)

add_library(ekfvio_klt_tracker include/ekf_vio/KLTTracker.cpp)
target_link_libraries(ekfvio_klt_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} vio_tc_ekf ekfvio_Params ekfvio_Feature ekfvio_Frame)

add_library(ekfvio_full include/ekf_vio/VIO.h include/ekf_vio/VIO.cpp)
target_link_libraries(ekfvio_full ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_Params ekfvio_Feature ekfvio_Frame vio_tc_ekf ekfvio_klt_tracker)

add_executable(ekfvio_node src/invio_node.cpp)
target_link_libraries(ekfvio_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_full)

add_executable(ekfvio_jacobian_test test/jacobian_test.cpp)
target_link_libraries(ekfvio_jacobian_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_full)

add_executable(ekfvio_ekf_test test/test_ekf.cpp)
target_link_libraries(ekfvio_ekf_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_full)

add_executable(ekfvio_klt_test test/klt_test.cpp)
target_link_libraries(ekfvio_klt_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_full)

add_executable(ekfvio_analyze_ekf test/analyzeEKFSimulation.cpp)
target_link_libraries(ekfvio_analyze_ekf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${sophus_LIBRARIES} ekfvio_full)
