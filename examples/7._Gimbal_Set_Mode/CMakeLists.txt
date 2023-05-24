cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        gimbal_set_mode.cpp
        ${SOURCES})

add_executable(GimbalSetMode
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(GimbalSetMode
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )