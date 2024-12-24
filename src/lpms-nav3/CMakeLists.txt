cmake_minimum_required(VERSION 3.8)
project(lpms-nav3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c23")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Os -Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)


file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

ament_auto_add_library (
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${UDEV_INCLUDE_DIRS})
link_directories(${UDEV_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${UDEV_LIBRARIES})

ament_auto_package()
