cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        move_gimbal_speed_mode.cpp
        ${SOURCES})

add_executable(MoveGimbalSpeedMode
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(MoveGimbalSpeedMode
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )