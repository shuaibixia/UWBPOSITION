cmake_minimum_required(VERSION 3.10)
project(PsdkController CXX)

# 设置编译标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 PSDK 根路径
set(PSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../Payload-SDK-3.11.1)

# 自动检测当前系统架构（arm64 或 x86_64）
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Detected architecture: ${ARCHITECTURE}")

# 根据架构选择正确的库目录
if(${ARCHITECTURE} STREQUAL "aarch64")
    set(PSDK_LIB_DIR ${PSDK_ROOT_DIR}/psdk_lib/lib/aarch64-linux-gnu-gcc)
elseif(${ARCHITECTURE} STREQUAL "armv7l")
    set(PSDK_LIB_DIR ${PSDK_ROOT_DIR}/psdk_lib/lib/arm-linux-gnueabihf-gcc)
else()
    set(PSDK_LIB_DIR ${PSDK_ROOT_DIR}/psdk_lib/lib/x86_64-linux-gnu-gcc)
endif()

# 包含头文件路径
include_directories(
    ${PSDK_ROOT_DIR}/psdk_lib/include
    ${PSDK_ROOT_DIR}/include
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/include
    ${PSDK_ROOT_DIR}/samples/sample_c++/common
)

# 添加可执行文件
add_executable(TakeOffDemo src/TakeOffDemo.cpp)

# 链接库
target_link_libraries(TakeOffDemo
    ${PSDK_LIB_DIR}/libpayloadsdk.a
    pthread
    m
    dl
)