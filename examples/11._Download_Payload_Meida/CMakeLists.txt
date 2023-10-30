cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17  -Wall -Werror")

set(APP_SOURCES 
        download_payload_media.cpp
        ${SOURCES})

add_executable(DownloadPayloadMedia
               ${APP_SOURCES}
               )

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})
find_package(CURL REQUIRED) 

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(DownloadPayloadMedia
                    ${JSONCPP_LIBRARIES} 
                    ${CURL_LIBRARIES}
                    )