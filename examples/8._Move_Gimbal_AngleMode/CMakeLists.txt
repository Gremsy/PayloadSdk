cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        move_gimbal_angle_mode.cpp
        ${SOURCES})

add_executable(MoveGimbalAngleMode
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(MoveGimbalAngleMode
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )