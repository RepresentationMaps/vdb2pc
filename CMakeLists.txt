#####################################################################
 #
 # Software License Agreement
 #
 #  Copyright (c) 2018, Simbe Robotics, Inc.
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   # Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   # Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   # Neither the name of Simbe Robotics, Inc. nor the names of its
 #     contributors may be used to endorse or promote products derived
 #     from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 #  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 #  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 #  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 #  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 #  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 #  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 #  POSSIBILITY OF SUCH DAMAGE.
 #
 # Author: Steve Macenski (steven.macenski@simberobotics.com)
 # Purpose: convert native vdb files to pointclouds
 #####################################################################/

cmake_minimum_required(VERSION 3.8)
project(vdb2pc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenVDB REQUIRED)

# Specify additional locations of header files
include_directories(
  include/vdb2pc
  ${OpenVDB_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(vdb2pc_lib src/vdb2pc.cpp)
add_library(vdb2pc_pub_lib src/vdb2pc_publisher.cpp)

# Link against necessary libraries
ament_target_dependencies(vdb2pc_lib rclcpp)
ament_target_dependencies(vdb2pc_lib pcl_conversions sensor_msgs)

# Add the executable target (TEST)
add_executable(test_node src/main_test.cpp)

# Link the executable against the library
target_link_libraries(vdb2pc_lib ${Tbb_LIBRARIES} ${OpenVDB_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(vdb2pc_pub_lib vdb2pc_lib)
target_link_libraries(test_node vdb2pc_pub_lib)

# Install the libraries
install(
  TARGETS vdb2pc_lib
  EXPORT vdb2pc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS vdb2pc_pub_lib
  EXPORT vdb2pc_pub_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the executables
install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(
  DIRECTORY include/vdb2pc
  DESTINATION include
)

# Export libraries
ament_export_targets(vdb2pc_lib HAS_LIBRARY_TARGET)
ament_export_targets(vdb2pc_pub_lib HAS_LIBRARY_TARGET)
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
