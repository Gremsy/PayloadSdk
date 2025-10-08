cmake_minimum_required(VERSION 3.5)
project(ui_demo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
pkg_check_modules(GSTREAMER REQUIRED 
    gstreamer-1.0 
    gstreamer-video-1.0 
    gstreamer-app-1.0
)

include_directories(${GTKMM_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/ui_gtk
                    ${CMAKE_CURRENT_SOURCE_DIR}/ui_gtk/PayloadControlTab
                    )

link_directories(${GTKMM_LIBRARY_DIRS} ${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GTKMM_CFLAGS_OTHER} ${GSTREAMER_CFLAGS_OTHER})

file(GLOB FILE_SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/ui_gtk/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ui_gtk/PayloadControlTab/*.cpp
        )

add_executable(${PROJECT_NAME} 
                main.cpp
                ${FILE_SOURCES}
                )
target_link_libraries(  ${PROJECT_NAME} 
                        PayloadSDK
                        pthread
                        ${GTKMM_LIBRARIES}
                        ${GSTREAMER_LIBRARIES}
                        )