cmake_minimum_required(VERSION 3.5)

set(EXAMPLES
    connect.cpp
    load_payload_settings.cpp
    change_payload_settings.cpp
    capture_image.cpp
    record_video.cpp
    get_video_streaming.cpp
    move_gimbal_angle_mode.cpp
    move_gimbal_speed_mode.cpp
    get_payload_status.cpp
    set_camera_zoom_focus.cpp
    gimbal_set_mode.cpp
    set_video_source.cpp
)

find_package( Threads )

# Now SUBMODULE_VAR is available here
message(STATUS "SUBMODULE_VAR = ${PAYLOADSDK_DIR}")
  
#include 
include_directories(${PAYLOADSDK_DIR}
                    ${PAYLOADSDK_DIR}/third-party/mavlink/include
                    ${PAYLOADSDK_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDK_DIR}/gSDK/src
                    ${GST_INCLUDE_DIRS}
                    )

#
# build examples 
foreach(entry ${EXAMPLES})
  string(REPLACE ".cpp" "" target ${entry})
  add_executable(${target} ${target}.cpp ${SOURCES})

  target_link_libraries(${target}
                     ${PAYLOADSDK_LIB}/libPayloadSDK.a
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     stdc++fs
                     )

endforeach()


# build Downlad media files example
add_executable(DownloadMediaFiles
               download_media_files.cpp
               )

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED) 

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(DownloadMediaFiles
                    ${JSONCPP_LIBRARIES} 
                    ${CURL_LIBRARIES}
                    )

