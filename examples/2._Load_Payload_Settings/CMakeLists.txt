cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        load_payload_settings.cpp
        ${SOURCES})

add_executable(LoadPayloadSettings
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(LoadPayloadSettings
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )