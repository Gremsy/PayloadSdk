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