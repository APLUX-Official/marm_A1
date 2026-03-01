cmake_minimum_required(VERSION 3.5)

project(A1_description)

# ROS1
# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

#ROS2
find_package(ament_cmake REQUIRED)
install(
	DIRECTORY launch meshes urdf rviz
	DESTINATION share/${PROJECT_NAME}
	)
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()