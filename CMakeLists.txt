cmake_minimum_required(VERSION 3.5)

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

find_package(PkgConfig REQUIRED)
#Use Pkg-config to configure GStreamer
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GST REQUIRED
                    gstreamer-1.0
                    gstreamer-video-1.0
                    gstreamer-app-1.0
                    )

# Set library path based on system architecture
set(PAYLOADSDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(PAYLOADSDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(PAYLOADSDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/libs/aarch64)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message("Processor detected: ${CMAKE_SYSTEM_PROCESSOR}")
message("Path to library: ${PAYLOADSDK_LIB}")

# examples
add_subdirectory(examples)


# Project subdirectory
if (VIO)
add_subdirectory(sample_project/demo_project)
endif(VIO)
