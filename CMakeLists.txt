cmake_minimum_required(VERSION 3.16)

find_package(Threads REQUIRED)

add_library(lidar_library STATIC
    lidar_library.cpp
    lidar_library.hpp
)

target_include_directories(lidar_library PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(lidar_library
    Threads::Threads
)

if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(lidar_library PUBLIC "-pthread")
endif()

