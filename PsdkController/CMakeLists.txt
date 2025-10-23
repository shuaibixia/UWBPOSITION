# ---------------------------------------------------------------
# 这是你的项目编译文件: ~/UwbProject/PsdkController/CMakeLists.txt
# (已修正为 PsdkService -> TakeOff)
# ---------------------------------------------------------------

# 1. CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(PsdkController)

# 2. 找到 PSDK 库 (它在我们旁边的文件夹)
set(PSDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Payload-SDK-3.11.1)
message(STATUS "PSDK Root found at: ${PSDK_ROOT_DIR}")

# 3. 包含 PSDK 的头文件
include_directories(
    ${PSDK_ROOT_DIR}/include
    ${PSDK_ROOT_DIR}/hal
    ${PSDK_ROOT_DIR}/utils/include
    ${PSDK_ROOT_DIR}/platform/linux/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src # <-- 包含我们自己的 src 目录以找到 app_info.h
)

# 4. (新) 定义编译选项 (在这里设置 PSDK 版本和串口)
add_compile_definitions(
    PSDK_ARCH_ARM_64
    PSDK_VERSION_MAJOR=3
    PSDK_VERSION_MINOR=11
    PSDK_VERSION_PATCH=1
    LINUX
    PSDK_COM_PORT_MAX_NUM=2
    PSDK_COM_PORT_0_NAME="\"/dev/ttyUSB0\""   # (!! 关键 !!) 检查这是否是你的预期串口
    PSDK_COM_PORT_0_BAUD_RATE=460800       # (!! 关键 !!) 波特率
)

# 5. 定义我们的可执行程序
#    (已修正为 TakeOff，匹配你的 TakeOff.cpp)
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(TakeOff ${SOURCES})

# 6. 指定 PSDK 库文件的位置 (我们在第 1 步编译好的)
link_directories(${PSDK_ROOT_DIR}/build/lib)

# 7. 链接我们的程序到 PSDK 库
#    (已修正为 TakeOff)
target_link_libraries(TakeOff
    psdk_v3       # PSDK 3.x 核心库
    psdk_utils
    psdk_hal
    pthread       # Linux 线程库
    rt            # Linux 实时库
)

# 8. (可选但推荐) 安装到 bin 目录
#    (已修正为 TakeOff)
install(TARGETS TakeOff DESTINATION "bin")