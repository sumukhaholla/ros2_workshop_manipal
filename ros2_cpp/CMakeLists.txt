cmake_minimum_required(VERSION 3.8)
project(ros2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ros2_msgs REQUIRED)

find_package(ompl REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED) 
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(RGBD_seg src/RGBD_seg.cpp)
ament_target_dependencies(RGBD_seg rclcpp sensor_msgs message_filters pcl_conversions visualization_msgs Eigen3 tf2 tf2_ros tf2_eigen)
target_link_libraries(RGBD_seg ${PCL_LIBRARIES})

add_executable(RGBD_segV02 src/RGBD_segV02.cpp)
ament_target_dependencies(RGBD_segV02 rclcpp sensor_msgs message_filters pcl_conversions visualization_msgs Eigen3 tf2 tf2_ros tf2_eigen ros2_msgs)
target_link_libraries(RGBD_segV02 ${PCL_LIBRARIES})

add_executable(forward_kinematics src/forward_kinematics.cpp)
ament_target_dependencies(forward_kinematics rclcpp sensor_msgs message_filters pcl_conversions visualization_msgs Eigen3 tf2 tf2_ros tf2_eigen ros2_msgs moveit_ros_planning_interface moveit_core moveit_ros_planning moveit_visual_tools)
target_link_libraries(forward_kinematics ${PCL_LIBRARIES})

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics rclcpp sensor_msgs message_filters pcl_conversions visualization_msgs Eigen3 tf2 tf2_ros tf2_eigen ros2_msgs moveit_ros_planning_interface moveit_core moveit_ros_planning moveit_visual_tools)
target_link_libraries(inverse_kinematics ${PCL_LIBRARIES})

add_executable(moveit_planner src/moveit_planner.cpp)
ament_target_dependencies(moveit_planner rclcpp sensor_msgs message_filters pcl_conversions visualization_msgs Eigen3 tf2 tf2_ros tf2_eigen ros2_msgs moveit_ros_planning_interface moveit_core moveit_ros_planning moveit_visual_tools linkattacher_msgs)
target_link_libraries(moveit_planner ${PCL_LIBRARIES})

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp ros2_msgs)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp ros2_msgs)

install(TARGETS
  RGBD_seg
  RGBD_segV02
  forward_kinematics
  inverse_kinematics
  moveit_planner
  service_server
  service_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
