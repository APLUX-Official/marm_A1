cmake_minimum_required(VERSION 3.8)
project(moveit_controller_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)  # Added required keyword
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rosidl_default_generators  REQUIRED)
# # 添加服务文件（注意修改路径为你实际的路径）
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SetArmAction.srv"
#   "srv/SetArmJointValues.srv"
#   "srv/SetArmTargetPose.srv"
#   DEPENDENCIES std_msgs geometry_msgs  # 如果有依赖其他消息包，列出它们
# )

add_executable(moveit_controller_server_node src/moveit_controller_server.cpp)
ament_target_dependencies(moveit_controller_server_node
  rclcpp
  std_msgs
  geometry_msgs
  custom_interface
  moveit_ros_planning_interface
  moveit_ros_planning
)

# 链接生成的接口 - 使用新的推荐方式
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(moveit_controller_server_node "${cpp_typesupport_target}")

add_executable(moveit_joint_states_repub src/moveit_joint_states_repub.cpp)
ament_target_dependencies(moveit_joint_states_repub
  rclcpp
  sensor_msgs
)

add_executable(marm_demo src/marm_demo.cpp)
ament_target_dependencies(marm_demo
  rclcpp
  std_msgs
  geometry_msgs
  custom_interface
  tf2_ros
  moveit_ros_planning_interface
  moveit_ros_planning
  ament_index_cpp
)

add_executable(arm_test src/arm_test.cpp)
ament_target_dependencies(arm_test
  rclcpp
  std_msgs
  geometry_msgs
  custom_interface
  tf2_ros
  moveit_ros_planning_interface
  moveit_ros_planning
)
# 链接yaml-cpp库
target_link_libraries(marm_demo yaml-cpp)

# 链接生成的接口 - 使用新的推荐方式
# target_link_libraries(marm_demo "${cpp_typesupport_target}")

# 新增a2_control_behaviors.cpp的可执行文件和依赖
# add_executable(a2_control_behaviors src/a2_control_behaviors.cpp)
# ament_target_dependencies(a2_control_behaviors
#   rclcpp
#   std_msgs
#   geometry_msgs
#   custom_interface
#   tf2_ros
# )

install(TARGETS
moveit_controller_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
moveit_joint_states_repub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
marm_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
arm_test
  DESTINATION lib/${PROJECT_NAME}
)

# 添加test demo可执行文件
add_executable(moveit_test_demo src/test.cpp)
ament_target_dependencies(moveit_test_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
moveit_test_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
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
