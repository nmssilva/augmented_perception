cmake_minimum_required(VERSION 2.8.11)
project(calibration_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  colormap
  roslib
  image_transport
  cv_bridge
  rviz
  velodyne_pointcloud
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs colormap roslib velodyne_pointcloud
)


find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(lidar_segmentation REQUIRED)
if ( NOT lidar_segmentation_FOUND )
	message(FATAL_ERROR "Package lidar_segmentation required but not found!")
endif( NOT lidar_segmentation_FOUND )


link_directories(
    ${GTKMM_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    )

include_directories(${catkin_INCLUDE_DIRS}
	            ${PCL_INCLUDE_DIRS}
		    ${OpenCV_INCLUDE_DIRS}
		    ${roscpp_INCLUDE_DIRS}
		    ${lidar_segmentation_INCLUDE_DIRS}
		    include
                   )

find_library(FLYCAPTURE2 flycapture)

##############################################################################
# Qt
##############################################################################

# References:
# http://doc.qt.io/qt-5/cmake-manual.html
# http://www.cmake.org/cmake/help/v3.0/manual/cmake-qt.7.html
# http://www.kdab.com/using-cmake-with-qt-5/
# http://rdu.im/ros/programming/2014/03/25/using-qt-in-ros-catkin-packages.html
# http://stackoverflow.com/questions/25989448/implementing-qt-project-through-cmake
	# https://github.com/Gluttton/SOS/tree/master/25989448

set (MyProject_src src/gui_main.cpp
		src/gui_mainwindow.cpp
		src/gui_myrviz.cpp
		src/gui_calibration_node.cpp
		src/calibration_utils.cpp
		src/visualization_rviz_calibration.cpp
		src/gui_options.cpp
		src/gui_supportedsensors.cpp
		src/gui_QProgressIndicator.cpp
		)
set (MyProjectLib_hdr include/calibration_gui/gui_mainwindow.h
			include/calibration_gui/gui_myrviz.h
			include/calibration_gui/gui_calibration_node.h
			include/calibration_gui/gui_options.h
			include/calibration_gui/gui_supportedsensors.h
			include/calibration_gui/gui_QProgressIndicator.h
			)

set (MyProjectLib_ui  ui/mainwindow.ui
			ui/options.ui
			)

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})

##############################################################################
# Binaries
##############################################################################

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  QT4_WRAP_UI(MyProjectLib_ui_moc ${MyProjectLib_ui})
  QT4_WRAP_CPP(MyProjectLib_hdr_moc ${MyProjectLib_hdr})
  set(QT_LIBRARIES ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5Widgets REQUIRED)
  QT5_WRAP_UI(MyProjectLib_ui_moc ${MyProjectLib_ui})
  QT5_WRAP_CPP(MyProjectLib_hdr_moc ${MyProjectLib_hdr})
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_executable(calibration_gui ${MyProject_src} ${MyProjectLib_ui_moc} ${MyProjectLib_hdr_moc})
target_link_libraries (calibration_gui ${QT_LIBRARIES}
					${catkin_LIBRARIES}
					)


add_executable(sick_ldmrs src/visualization_rviz_ldmrs.cpp src/sick_ldmrs.cpp src/common_functions.cpp)

target_link_libraries(sick_ldmrs ${catkin_LIBRARIES}
			         ${PCL_LIBRARIES}
				 ${lidar_segmentation_LIBRARIES}
				 ${roscpp_LIBRARIES}
				)


add_executable(sick_lms151 src/visualization_rviz_lms.cpp src/sick_lms151.cpp src/common_functions.cpp)

target_link_libraries(sick_lms151 ${catkin_LIBRARIES}
				    ${PCL_LIBRARIES}
				    ${lidar_segmentation_LIBRARIES}
				    ${roscpp_LIBRARIES}
					)


#add_executable(swissranger src/swissranger.cpp src/visualization_rviz_swissranger.cpp)

#target_link_libraries(swissranger ${catkin_LIBRARIES}
#				  ${PCL_LIBRARIES}
#				  ${lidar_segmentation_LIBRARIES}
#				  ${roscpp_LIBRARIES}
#					)


add_executable(kinect src/kinect.cpp src/visualization_rviz_kinect.cpp)

target_link_libraries(kinect ${catkin_LIBRARIES}
				  ${PCL_LIBRARIES}
				  ${lidar_segmentation_LIBRARIES}
				  ${roscpp_LIBRARIES}
					)


add_executable(point_grey_camera src/point_grey_camera.cpp)

target_link_libraries(point_grey_camera ${catkin_LIBRARIES}
			     		${OpenCV_LIBS}
            				${roscpp_LIBRARIES}
                     			)

add_executable(velodyne_vlp16_node src/velodyne_vlp16.cpp src/common_functions.cpp src/visualization_rviz_velodyne.cpp )

target_link_libraries(velodyne_vlp16_node ${catkin_LIBRARIES}
				    ${PCL_LIBRARIES}
				    ${lidar_segmentation_LIBRARIES}
				    ${roscpp_LIBRARIES}
					)

