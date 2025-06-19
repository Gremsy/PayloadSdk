cmake_minimum_required(VERSION 3.8)

project(PayloadSDK)

set(CMAKE_BUILD_TYPE Debug)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
    # Remove -Wpedantic to avoid issues with mavlink library
    # Add flags to suppress warnings from third-party mavlink code
    add_compile_options(-Wno-missing-field-initializers)
    add_compile_options(-Wno-unused-parameter)
    add_compile_options(-Wno-unused-variable)
    add_compile_options(-Wno-unused-function)
    add_compile_options(-Wno-address-of-packed-member)
    add_compile_options(-Wno-format)
    add_compile_options(-Wno-error)  # Treat all warnings as warnings, not errors
endif()

# Find ROS2 dependencies FIRST (before other packages)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find other dependencies
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Use Pkg-config to configure GStreamer
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-video-1.0
    gstreamer-app-1.0
)

pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED)

# Set up include directories
set(PAYLOADSDKLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(
    ${PAYLOADSDKLIB_DIR}
    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
    ${PAYLOADSDKLIB_DIR}/gSDK/src
    ${GST_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIR}
)

# Add subdirectories for libraries
add_subdirectory(libs)

# Add examples subdirectory
add_subdirectory(examples)

# Project subdirectories
if (VIO)
    add_subdirectory(tests/demo_project)
endif(VIO)
if (GHADRON)
    add_subdirectory(tests/ghadron_project)
endif(GHADRON)

# Create the main executable with descriptive name
add_executable(gremsysdk
    src/gremsy_heading_follower.cpp
)

# Set target dependencies for ROS2 packages
ament_target_dependencies(gremsysdk
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
)

# Link libraries
target_link_libraries(gremsysdk
    PayloadSDK
    Threads::Threads
    ${GST_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${CURL_LIBRARIES}
    stdc++fs
)

# Set specific compile options for the main executable to be more lenient with SDK
target_compile_options(gremsysdk PRIVATE
    -Wno-error
    -Wno-missing-field-initializers
    -Wno-unused-parameter
)

# Install the executable
install(TARGETS gremsysdk
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

# Install package.xml
install(FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # Skip copyright checks for now
    set(ament_cmake_copyright_FOUND TRUE)
    # Skip cpplint checks for now
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()