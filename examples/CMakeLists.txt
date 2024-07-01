cmake_minimum_required(VERSION 3.5)

if (VIO)
set(EXAMPLES
    connect.cpp
    load_payload_settings.cpp
    change_payload_settings.cpp
    eo_capture_image.cpp
    eo_record_video.cpp
    get_video_streaming.cpp
    move_gimbal_angle_mode.cpp
    move_gimbal_speed_mode.cpp
    get_payload_status.cpp
    eo_set_camera_zoom_focus.cpp
    gimbal_set_mode.cpp
    ir_set_camera_zoom.cpp
    ir_capture_image.cpp
    ir_record_video.cpp
    ir_set_palete.cpp
    download_media_files.cpp
    do_object_tracking.cpp
    time_lapse_photography.cpp
)
endif(VIO)

if (ZIO)
set(EXAMPLES
    connect.cpp
    load_payload_settings.cpp
    change_payload_settings.cpp
    eo_capture_image.cpp
    eo_record_video.cpp
    get_video_streaming.cpp
    move_gimbal_angle_mode.cpp
    move_gimbal_speed_mode.cpp
    get_payload_status.cpp
    eo_set_camera_zoom_focus.cpp
    gimbal_set_mode.cpp
    # ir_set_camera_zoom.cpp
    # ir_capture_image.cpp
    # ir_record_video.cpp
    # ir_set_palete.cpp
    download_media_files.cpp
    # do_object_tracking.cpp
    time_lapse_photography.cpp
)
endif(ZIO)

if (GHADRON)
set(EXAMPLES
    connect.cpp
    load_payload_settings.cpp
    # change_payload_settings.cpp
    eo_capture_image.cpp
    eo_record_video.cpp
    # get_video_streaming.cpp
    move_gimbal_angle_mode.cpp
    move_gimbal_speed_mode.cpp
    # get_payload_status.cpp
    eo_set_camera_zoom_focus.cpp
    gimbal_set_mode.cpp
    ir_set_camera_zoom.cpp
    ir_capture_image.cpp
    ir_record_video.cpp
    ir_set_palete.cpp
    download_media_files.cpp
    # time_lapse_photography.cpp
)
endif(GHADRON)
find_package( Threads )

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED)

# Now SUBMODULE_VAR is available here
message(STATUS "SUBMODULE_VAR = ${PAYLOADSDK_DIR}")
  
#include 
include_directories(${PAYLOADSDK_LIB}
                    ${PAYLOADSDK_LIB}/payload-define
                    ${PAYLOADSDK_DIR}/third-party/mavlink/include
                    ${PAYLOADSDK_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDK_DIR}/gSDK/src
                    ${GST_INCLUDE_DIRS}
                    ${CURL_INCLUDE_DIR}
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
                     ${JSONCPP_LIBRARIES}
                     ${CURL_LIBRARIES}
                     stdc++fs
                     )

endforeach()

