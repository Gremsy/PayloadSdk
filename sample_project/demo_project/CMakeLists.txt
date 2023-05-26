cmake_minimum_required(VERSION 3.5)
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME ProjectName)  
project(${ProjectName} LANGUAGES CXX)


###source for project
set(APP_SOURCES
	main.cpp
	${SOURCES}
	)
#include 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# 
add_executable(${PROJECT_NAME}
               ${APP_SOURCES}
               )
find_package( Threads )
target_link_libraries(${PROJECT_NAME}
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     ${OpenCV_LIBS}
                     ${CAIRO_LIBRARIES}
                     stdc++fs
#                      config++ 
                     )

