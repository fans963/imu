cmake_minimum_required(VERSION 3.8)
project(visual)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c23")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")

set(CMAKE_THREAD_LIBS_INIT "-lpthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Os -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_auto_add_executable (
  ${PROJECT_NAME}_exe
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_exe
 rclcpp 
 std_msgs
 tf2
 tf2_ros
 sensor_msgs
 lpms_nav3
)

target_link_libraries(${PROJECT_NAME}_exe
 spdlog::spdlog
 yaml-cpp
)

include_directories(${PROJECT_SOURCE_DIR}/include)

install(DIRECTORY include/
  DESTINATION include
)

ament_auto_package()