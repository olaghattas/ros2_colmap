cmake_minimum_required(VERSION 3.8)
project(colmap_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_colmap_wrapper REQUIRED)
find_package(COLMAP REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})
set(dependencies
        rclcpp
        ament_index_cpp
        )
add_executable(hello_world src/hello_world.cpp)
ament_target_dependencies(hello_world ${dependencies})

set(COLMAP_INSTALL_PATH "/home/ola/smart_home/build/ros2_colmap_wrapper/COLMAP-prefix/src/COLMAP-build/src/colmap")
target_link_libraries(hello_world ${COLMAP_LIBRARIES} ${ros2_colmap_wrapper_LIBRARIES} ) #${COLMAP_INSTALL_PATH}/libcolmap.a)

#install(
#        DIRECTORY image_registering
#        DESTINATION share/colmap_localization
#)

ament_package()
