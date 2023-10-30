cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        get_status_gimbal.cpp
        ${SOURCES})

add_executable(GetStatusGimbal
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(GetStatusGimbal
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )