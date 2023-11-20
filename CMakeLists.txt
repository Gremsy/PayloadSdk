cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)

set(CMAKE_BUILD_TYPE Debug)

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
add_subdirectory(sample_project/demo_project)
