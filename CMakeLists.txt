cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_image_mesh)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs sensor_msgs shape_msgs rviz pluginlib class_loader cv_bridge message_generation geometry_msgs sensor_msgs cmake_modules eigen_conversions)
find_package(Eigen REQUIRED)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

## I (original author) prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

include_directories(SYSTEM "msg_gen/cpp/include")

add_message_files(
  FILES
  TexturedQuad.msg
  TexturedQuadArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
  LIBRARIES
    rviz_plugin_mesh_display_custom
    ${OPENGL_LIBRARIES}
  INCLUDE_DIRS
  	src
    ${OPENGL_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp 
    rospy
    roslib
    std_msgs 
    sensor_msgs
    shape_msgs
    rviz 
    pluginlib 
    class_loader
    cv_bridge
  	geometry_msgs
  	sensor_msgs
  	eigen_conversions
)


include_directories(
	src
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${OPENGL_INCLUDE_DIR}
                    ${QT_INCLUDE_DIR})
                    
link_directories(${catkin_LIBRARY_DIRS})


# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES src/mesh_display_custom.h)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

set(VIGIR_MESH_LIB_NAME rviz_plugin_mesh_display_custom)

add_library(${VIGIR_MESH_LIB_NAME}_core src/mesh_display_custom.cpp ${MOC_SOURCES})
target_link_libraries(${VIGIR_MESH_LIB_NAME}_core ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Eigen_LIBRARIES})

add_dependencies(${VIGIR_MESH_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${VIGIR_MESH_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${VIGIR_MESH_LIB_NAME} ${VIGIR_MESH_LIB_NAME}_core ${catkin_LIBRARIES})

add_dependencies(${VIGIR_MESH_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${VIGIR_MESH_LIB_NAME} ${VIGIR_MESH_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY textures
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_plugin_mesh_display_custom
)

install(FILES
  mesh_display_custom_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# Test 1: Single image
add_executable(single_image_test tests/single_image_test.cpp ${SOURCE_FILES} ${HEADER_FILES})

add_dependencies(single_image_test ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(single_image_test
  ${catkin_LIBRARIES}
)
