cmake_minimum_required(VERSION 3.2)
project(waypoint_navigation_plugin)
find_package(catkin REQUIRED COMPONENTS rviz rosbag std_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})

else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
  # set variable names already used with Qt4
  set(QT_LIBRARIES Qt5::Widgets)
  set(QTVERSION ${Qt5Widgets_VERSION})
endif()

## Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math -Werror=return-type")

if(rviz_QT_VERSION VERSION_LESS "5")

  # Header files that need Qt Moc pre-processing for use with Qt signals, etc:
  qt4_wrap_cpp(MOC_FILES
          src/waypoint_nav_frame.h
          src/waypoint_nav_tool.h
  )

  # Convert the Qt Signals and Slots for QWidget events
  qt4_wrap_ui(UIC_FILES
          ui/WaypointNavigation.ui
  )
else()
  # Header files that need Qt Moc pre-processing for use with Qt signals, etc:
  qt5_wrap_cpp(MOC_FILES
          src/waypoint_nav_frame.h
          src/waypoint_nav_tool.h
  )

  # Convert the Qt Signals and Slots for QWidget events
  qt5_wrap_ui(UIC_FILES
          ui/WaypointNavigation.ui
  )
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
        src/waypoint_nav_frame.cpp
        src/waypoint_nav_tool.cpp
  ${MOC_FILES}
)

set(LIB_NAME waypoint_nav_plugin)
add_library(${LIB_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES} ${QT_LIBRARIES})

## Install rules

install(DIRECTORY .
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Install rules
install(FILES
  #plugin_description.xml
  #camera_plugin_description.xml
        waypoint_nav_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
