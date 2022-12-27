cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)

set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# # iclude OpenTLD
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# include(OpenTLD)


# # find_package(vpi REQUIRED)

# find_package( OpenCV REQUIRED )
# if(OpenCV_FOUND)
#    message("Found OpenCV")
#    message("Includes: " ${OpenCV_INCLUDE_DIRS})
# endif(OpenCV_FOUND)


find_package(PkgConfig REQUIRED)
#Use Pkg-config to configure GStreamer
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GST REQUIRED
                    gstreamer-1.0
                    gstreamer-video-1.0
                    gstreamer-app-1.0
                    )

#Include header files
include_directories(
            ${GST_INCLUDE_DIRS}
            ${CAIRO_INCLUDE_DIRS}
            ${OPENTLD_INCLUDES}
            ${OpenCV_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include/gimbal
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include/mavlink/include/mavlink/v2.0

        )

set(SOURCES   
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/autopilot_interface.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/mavlink_ftp.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/serial_port.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/udp_port.cpp  
            ${CMAKE_CURRENT_SOURCE_DIR}/payloadSdkInterface.cpp          
            )

add_subdirectory(examples/1._Connect_to_Payload)
add_subdirectory(examples/2._Load_Payload_Settings)
add_subdirectory(examples/3._Set_Payload_Settings)
add_subdirectory(examples/4._Capture_Image)