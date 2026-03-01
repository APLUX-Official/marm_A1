# Test executables configuration
cmake_minimum_required(VERSION 3.8)

# Find OpenCV (must be built with GStreamer support)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUVC REQUIRED libuvc)

# Add test executable
add_executable(test_1 test_1.cpp)

# Include directories
target_include_directories(test_1 PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUVC_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(test_1
  ${OpenCV_LIBRARIES}
  ${LIBUVC_LIBRARIES}
)

# Set C++ standard
target_compile_features(test_1 PRIVATE cxx_std_17)

# Install test executable (optional)
install(TARGETS test_1
  DESTINATION lib/${PROJECT_NAME}
)