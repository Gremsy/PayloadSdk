cmake_minimum_required(VERSION 3.5)

set(PAYLOADSDKLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)
# set(PAYLOADSDKLIB_COMPILED ${CMAKE_CURRENT_SOURCE_DIR}/../build/libs)

if (VIO)
set(EXAMPLES
    check_connect.cpp

    camera_load_settings.cpp
    camera_change_settings.cpp
    camera_eo_capture_image.cpp
    camera_eo_record_video.cpp
    camera_ir_do_ffc_control.cpp
    camera_ir_set_zoom.cpp
    camera_ir_capture_image.cpp
    camera_ir_record_video.cpp
    camera_ir_set_palette.cpp
    camera_eo_set_zoom_focus.cpp
    camera_time_lapse_photography.cpp
    camera_do_setzoom_individual.cpp

    gimbal_move_angle.cpp
    gimbal_move_speed.cpp
    gimbal_load_settings.cpp
    gimbal_change_settings.cpp
    gimbal_do_calib.cpp
    gimbal_set_mode.cpp
    
    payload_download_media_files.cpp
    payload_do_object_tracking.cpp
    payload_do_object_detection.cpp
    payload_set_gps.cpp
    payload_set_system_time.cpp
    payload_get_status.cpp
    payload_get_video_streaming.cpp
    payload_extract_tiff_file.cpp
)
endif(VIO)

if (ZIO)
set(EXAMPLES
    check_connect.cpp

    camera_load_settings.cpp
    camera_change_settings.cpp
    camera_eo_capture_image.cpp
    camera_eo_record_video.cpp
    # camera_ir_set_zoom.cpp
    # camera_ir_capture_image.cpp
    # camera_ir_record_video.cpp
    # camera_ir_set_palette.cpp
    camera_time_lapse_photography.cpp
    camera_eo_set_zoom_focus.cpp

    gimbal_move_angle.cpp
    gimbal_move_speed.cpp
    gimbal_set_mode.cpp
    
    payload_get_status.cpp
    payload_get_video_streaming.cpp
    payload_download_media_files.cpp
    # payload_do_object_tracking.cpp
)
endif(ZIO)

if (GHADRON)
set(EXAMPLES
    check_connect.cpp

    camera_load_settings.cpp
    camera_change_settings.cpp
    camera_eo_capture_image.cpp
    camera_eo_record_video.cpp
    camera_eo_set_zoom_focus.cpp
    camera_ir_set_zoom.cpp
    camera_ir_capture_image.cpp
    camera_ir_record_video.cpp
    camera_ir_set_palette.cpp
    camera_ir_set_isotherms.cpp
    camera_time_lapse_photography.cpp

    gimbal_move_angle.cpp
    gimbal_move_speed.cpp
    gimbal_set_mode.cpp
    
    payload_get_status.cpp
    payload_download_media_files.cpp
    payload_do_object_tracking.cpp
    payload_do_object_detection.cpp
    payload_get_video_streaming.cpp
    payload_extract_tiff_file.cpp
)
endif(GHADRON)

find_package( Threads )

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)

#include 
include_directories(${PAYLOADSDKLIB_DIR}
                    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
                    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDKLIB_DIR}/gSDK/src
                    ${GST_INCLUDE_DIRS}
                    ${CURL_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    )

# build examples 
foreach(entry ${EXAMPLES})
  string(REPLACE ".cpp" "" target ${entry})
  add_executable(${target} ${target}.cpp ${SOURCES})

  target_link_libraries(${target}
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${JSONCPP_LIBRARIES}
                     ${CURL_LIBRARIES}
                     stdc++fs
                     ${OpenCV_LIBS}
                     )
#   install(TARGETS
#           ${target}
#           RUNTIME DESTINATION usr/bin/q_payload/app-payloadsdk/example)

endforeach()