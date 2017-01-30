cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_vio)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cmake_modules
  tf
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)


include_directories(
	${OpenCV_INCLUDE_DIRS}
	include
	include/pauvsi_vio
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
   DEPENDS eigen opencv
)

add_library(keyframe include/pauvsi_vio/KeyFrame.h)
set_target_properties(keyframe PROPERTIES LINKER_LANGUAGE CXX)

add_library(visualmeasurement include/pauvsi_vio/Measurement.hpp)
set_target_properties(visualmeasurement PROPERTIES LINKER_LANGUAGE CXX)

add_library(viostate include/pauvsi_vio/VIOState.hpp)
set_target_properties(viostate PROPERTIES LINKER_LANGUAGE CXX)

add_library(vioekf include/pauvsi_vio/VIOEKF.cpp)

add_library(featureTracker include/pauvsi_vio/FeatureTracker.cpp)

add_library(point include/pauvsi_vio/Point.h)
set_target_properties(point PROPERTIES LINKER_LANGUAGE CXX)

add_library(feature include/pauvsi_vio/Feature.h)
set_target_properties(feature PROPERTIES LINKER_LANGUAGE CXX)

add_library(frame include/pauvsi_vio/Frame.cpp)

add_library(vio include/pauvsi_vio/vio.cpp include/pauvsi_vio/Motion.cpp include/pauvsi_vio/Draw.cpp include/pauvsi_vio/Triangulate.cpp include/pauvsi_vio/GaussNewton.cpp)

add_executable(pauvsi_vio src/pauvsi_vio.cpp)
target_link_libraries(visualmeasurement ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(viostate ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES} visualmeasurement)
target_link_libraries(vioekf ${catkin_LIBRARIES} ${Eigen_LIBRARIES} viostate visualmeasurement)
target_link_libraries(frame feature viostate point ${Eigen_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(featureTracker frame ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(feature point ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(keyframe frame)
target_link_libraries(vio frame ${catkin_LIBRARIES} featureTracker vioekf viostate visualmeasurement point keyframe)
target_link_libraries(pauvsi_vio ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} vio)

