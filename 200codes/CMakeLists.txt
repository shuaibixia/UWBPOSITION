cmake_minimum_required(VERSION 3.10)
project(UWBPro C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

# 强制使用 Linux 分支
add_definitions(-DLINUX)
add_definitions(-UWIN32)

# 头文件目录
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/csv
    ${CMAKE_SOURCE_DIR}/net
    ${CMAKE_SOURCE_DIR}/pos
    ${CMAKE_SOURCE_DIR}/vel
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/lib/ybase
)

# Boost
find_package(Boost REQUIRED COMPONENTS log)

# pthread
find_package(Threads REQUIRED)

# 源文件列表
set(SOURCE_FILES
    main.cpp

    csv/csv.cpp
    net/net.cpp
    pos/posMain.cpp
    vel/velMain.cpp

    lib/rcm.c
    lib/rcmIf.c
    lib/rn.c
)

add_executable(UWBPro ${SOURCE_FILES})

target_link_libraries(UWBPro PRIVATE
    Boost::log
    Threads::Threads
)

message(STATUS "Config OK. Now run: make -j4")