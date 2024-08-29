cmake_minimum_required(VERSION 3.0.2)
project(urban_projection)

add_compile_options(-std=c++14)


set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  image_transport
  message_generation
  cmake_modules
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(projector src/urban_projection/project.cpp)
target_link_libraries(projector ${catkin_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(projector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

