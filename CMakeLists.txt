cmake_minimum_required(VERSION 3.8)
project(vdb2pc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenVDB REQUIRED)

# Specify additional locations of header files
include_directories(
  include/vdb2pc
  ${TBB_INCLUDE_DIR}
  ${OpenVDB_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(vdb2pc_lib src/vdb2pc.cpp)

# Link against necessary libraries
ament_target_dependencies(vdb2pc_lib rclcpp)

# Link the executable against the library
target_link_libraries(vdb2pc_lib ${TBB_LIBRARIES} ${OpenVDB_LIBRARIES} ${PCL_LIBRARIES})


# Install the libraries
install(
  TARGETS vdb2pc_lib
  EXPORT vdb2pc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install include files
install(
  DIRECTORY include/vdb2pc
  DESTINATION include
)

# Export libraries
ament_export_targets(vdb2pc_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp pcl_ros pcl_conversions)

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
