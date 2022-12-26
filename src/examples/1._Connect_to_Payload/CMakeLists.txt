cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        connect.cpp
        ${SOURCES})

add_executable(ConnectPayload
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(ConnectPayload
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )