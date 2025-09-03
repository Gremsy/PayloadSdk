cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)

set(CMAKE_BUILD_TYPE Debug)

if (MB1)
set(MB1 $ENV{MB1})
message("-- MB1: ${MB1}")
add_definitions(-DMB1=${MB1})
endif(MB1)

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

if (ORUSL)
set(ORUSL $ENV{ORUSL})
message("-- ORUSL: ${ORUSL}")
add_definitions(-DVIO=${ORUSL})
endif(ORUSL)

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
add_subdirectory(tests/parse_thermal_stream)
endif(VIO)

if (ORUSL)
add_subdirectory(tests/demo_project)
add_subdirectory(tests/parse_thermal_stream)
endif(ORUSL)

if (MB1)
add_subdirectory(tests/mb1_project)
endif(MB1)