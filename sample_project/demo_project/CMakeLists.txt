cmake_minimum_required(VERSION 3.5)

project(demo_project LANGUAGES CXX)


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
                     stdc++fs
#                      config++ 
                     )

