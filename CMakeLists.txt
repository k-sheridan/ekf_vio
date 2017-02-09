cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_vio)

#set(G2O_FOUND "NO")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

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
	/usr/include/eigen3
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)


FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(
  ${G2O_INCLUDE_DIR}
  /usr/include/suitesparse # for cholmod
)
LIST(APPEND G2O_LIBRARIES
    ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
    ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} cholmod cxsparse
)

IF(G2O_FOUND)
message("\n\n${Green}FOUND G2O: ${ColourReset}${G2O_LIBRARIES}\n\n")
ENDIF(G2O_FOUND)

IF(NOT G2O_FOUND)
message("\n\n${Red}ERROR: DID NOT FIND G2O${ColourReset}\n\n")
ENDIF(NOT G2O_FOUND)

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

add_library(point include/pauvsi_vio/Point.cpp)

add_library(feature include/pauvsi_vio/Feature.cpp)

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
target_link_libraries(vio frame ${catkin_LIBRARIES} ${G2O_LIBRARIES} featureTracker vioekf viostate visualmeasurement point keyframe)
target_link_libraries(pauvsi_vio ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} vio)

