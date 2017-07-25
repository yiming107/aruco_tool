cmake_minimum_required(VERSION 2.8.3)
project(aruco_tool)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})

message( STATUS "CMAKE_PREFIX_PATH " ${CMAKE_PREFIX_PATH})

##set(OpenCV_INCLUDE_DIRS "/usr/local/include")
#find_package(OpenCV 3.2.0 EXACT)
find_package(OpenCV 3 REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	tf
	resource_retriever
	image_transport
	cv_bridge
	message_generation
)



add_message_files(
  DIRECTORY msg
  FILES
	ARMarker.msg
	ARMarkers.msg
)

generate_messages(DEPENDENCIES
	geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    message_runtime
)

include_directories(
   ${OpenCV_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS} include
)

message(STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})


add_executable(arucoDetectionTopView
        src/arucoDetectionTopView.cpp
)

add_executable(arucoCalibration
        src/arucoCalibration.cpp
)


message( STATUS "OpenCV_VERSION: " ${OpenCV_VERSION} )


target_link_libraries(arucoDetectionTopView
	${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

target_link_libraries(arucoCalibration
	${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_dependencies(arucoDetectionTopView
	${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(arucoCalibration
	${PROJECT_NAME}_generate_messages_cpp
)


install(TARGETS arucoCalibration
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS arucoDetectionTopView
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY Param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Param
)

install(DIRECTORY RawData/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/RawData
)

install(DIRECTORY GeneratedVideos/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/GeneratedVideos
)

