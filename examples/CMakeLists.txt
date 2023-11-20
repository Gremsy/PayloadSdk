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
    get_gimbal_status.cpp
    set_camera_zoom_focus.cpp
)

find_package( Threads )

#
# build examples 
foreach(entry ${EXAMPLES})
  string(REPLACE ".cpp" "" target ${entry})
  add_executable(${target} ${target}.cpp ${SOURCES})

  target_link_libraries(${target}
                     PayloadSDK
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

