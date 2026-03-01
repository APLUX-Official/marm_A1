cmake_minimum_required(VERSION 3.8)
project(capture_image)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 自动查找所有依赖
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

# 自动添加可执行文件（会自动处理 include 目录和依赖）
ament_auto_add_executable(
  capture_image_node
  src/main.cpp
  src/capture_image.cpp
)

ament_target_dependencies(capture_image_node
  rclcpp sensor_msgs cv_bridge OpenCV tf2 tf2_ros geometry_msgs image_transport std_msgs ament_index_cpp
)

# Link yaml-cpp library
target_link_libraries(capture_image_node yaml-cpp)

ament_auto_add_executable(aruco_object_pose_node src/aruco_object_pose_node.cpp)
ament_target_dependencies(aruco_object_pose_node 
  rclcpp 
  cv_bridge 
  OpenCV 
  sensor_msgs 
  tf2_ros 
  geometry_msgs 
  tf2
  ament_index_cpp
)

# RGB检测节点
ament_auto_add_executable(rgb_detect_pose_node src/rgb_detect_pose_node.cpp)
ament_target_dependencies(rgb_detect_pose_node 
  rclcpp 
  cv_bridge 
  OpenCV 
  sensor_msgs 
  tf2_ros 
  geometry_msgs 
  tf2
  std_msgs
  ament_index_cpp
)

# Link yaml-cpp library
target_link_libraries(rgb_detect_pose_node yaml-cpp)


# 自动安装头文件目录
install(DIRECTORY include/
  DESTINATION include
)

# 安装ost.yaml文件到share目录
install(FILES ost.yaml
  DESTINATION share/${PROJECT_NAME}
)



# 测试相关（保持不变）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 自动安装launch目录到share
ament_auto_package(INSTALL_TO_SHARE launch)