cmake_minimum_required(VERSION 3.10)
project(PsdkController CXX)

# 设置 PSDK 根路径
set(PSDK_ROOT_DIR ../Payload-SDK-3.11.1)

# 包含头文件路径
include_directories(
    ${PSDK_ROOT_DIR}/psdk_lib/include
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2
    ${PSDK_ROOT_DIR}/samples/sample_c++/common
)

# 添加可执行文件
add_executable(TakeOffDemo src/TakeOffDemo.cpp)

# 链接库（如果官方示例有库的话）
# target_link_libraries(TakeOffDemo <库名>)