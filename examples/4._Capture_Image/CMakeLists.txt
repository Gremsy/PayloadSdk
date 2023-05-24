cmake_minimum_required(VERSION 3.5)

set(APP_SOURCES 
        capture_image.cpp
        ${SOURCES})

add_executable(CaptureImage
               ${APP_SOURCES}
               )

find_package( Threads )
target_link_libraries(CaptureImage
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )