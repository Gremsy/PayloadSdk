cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        set_camera_zoom_focus.cpp
        ${SOURCES})

add_executable(SetCameraZoomFocus
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(SetCameraZoomFocus
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )