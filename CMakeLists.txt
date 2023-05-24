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

# camera examples
add_subdirectory(examples/1._Connect_to_Payload)
add_subdirectory(examples/2._Load_Payload_Settings)
add_subdirectory(examples/3._Set_Payload_Settings)
add_subdirectory(examples/4._Capture_Image)
add_subdirectory(examples/5._Record_Video)
add_subdirectory(examples/6._Get_Video_Streaming)

# gimbal control examples
add_subdirectory(examples/7._Gimbal_Set_Mode)