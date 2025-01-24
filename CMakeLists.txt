cmake_minimum_required(VERSION 3.8)

project (PayloadSDK)

set(CMAKE_BUILD_TYPE Debug)

if (GHADRON)
set(GHADRON $ENV{GHADRON})
message("-- GHADRON: ${GHADRON}")
add_definitions(-DGHADRON=${GHADRON})
endif(GHADRON)

if (VIO)
set(VIO $ENV{VIO})
message("-- VIO: ${VIO}")
add_definitions(-DVIO=${VIO})
endif(VIO)

if (ZIO)
set(ZIO $ENV{ZIO})
message("-- ZIO: ${ZIO}")
add_definitions(-DZIO=${ZIO})
endif(ZIO)

add_definitions(-DVIO=1)

find_package(PkgConfig REQUIRED)
#Use Pkg-config to configure GStreamer
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GST REQUIRED
                    gstreamer-1.0
                    gstreamer-video-1.0
                    gstreamer-app-1.0
                    )

add_subdirectory(libs)

# examples
add_subdirectory(examples)

# Project subdirectory
if (VIO)
add_subdirectory(tests/demo_project)
endif(VIO)
if (GHADRON)
add_subdirectory(tests/ghadron_project)
endif(GHADRON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED)

set(PAYLOADSDKLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${PAYLOADSDKLIB_DIR}
                    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
                    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDKLIB_DIR}/gSDK/src
                    ${GST_INCLUDE_DIRS}
                    ${CURL_INCLUDE_DIR}
                    )



add_executable(gremsysdk src/gremsy_ros2_interface_node.cpp ${SOURCES})
ament_target_dependencies(gremsysdk rclcpp std_msgs geometry_msgs)

target_link_libraries(gremsysdk
                    PayloadSDK
                    ${CMAKE_THREAD_LIBS_INIT}
                    ${GST_LIBRARIES} 
                    ${JSONCPP_LIBRARIES}
                    ${CURL_LIBRARIES}
                    stdc++fs
                    )

install(TARGETS gremsysdk DESTINATION lib/${PROJECT_NAME})


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
