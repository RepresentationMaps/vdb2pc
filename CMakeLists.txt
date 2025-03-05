cmake_minimum_required(VERSION 3.8)
project(vdb2pc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB")

find_package(ament_cmake_auto REQUIRED)
find_package(OpenVDB MODULE REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include/vdb2pc
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/vdb2pc.cpp
  src/vdb2pc_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_lib 
 ${OpenVDB_LIBRARIES} ${PCL_LIBRARIES})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_lib
)

install(
  DIRECTORY include/vdb2pc
  DESTINATION include
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
