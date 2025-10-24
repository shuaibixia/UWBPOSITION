# ---------------------------------------------------------------
# 这是你的项目编译文件: ~/UwbProject/PsdkController/CMakeLists.txt
# (已修正所有 Manifold2 依赖路径和源文件)
# ---------------------------------------------------------------

# 1. CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(PsdkController)

# 2. 找到 PSDK 库 (它在我们旁边的文件夹)
set(PSDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Payload-SDK-3.11.1)
message(STATUS "PSDK Root found at: ${PSDK_ROOT_DIR}")

# 3. 包含所有必需的头文件 (!!! 最终修正 !!!)
include_directories(
    # --- PSDK 核心 C 库路径 ---
    ${PSDK_ROOT_DIR}/psdk_lib/include
    ${PSDK_ROOT_DIR}/utils/include
    ${PSDK_ROOT_DIR}/platform/linux/include
    
    # --- PSDK 示例 C++ 封装层路径 (!! 关键 !!) ---
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/application
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/logger
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/common/osal
    ${PSDK_ROOT_DIR}/samples/sample_c++/application 

    # --- (THE FIX) 你找到的正确路径 ---
    ${PSDK_ROOT_DIR}/samples/sample_c++/module_sample/flight_controller 
    ${PSDK_ROOT_DIR}/samples/sample_c++/modules/flight_controller

    # --- 我们自己的 App Info 路径 ---
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 4. 定义编译选项
add_compile_definitions(
    PSDK_ARCH_ARM_64
    PSDK_VERSION_MAJOR=3
    PSDK_VERSION_MINOR=11
    PSDK_VERSION_PATCH=1
    LINUX
    PSDK_COM_PORT_MAX_NUM=2
    PSDK_COM_PORT_0_NAME="\"/dev/ttyUSB0\""
    PSDK_COM_PORT_0_BAUD_RATE=460800
)

# 5. (!!! 关键修正 !!!) 定义所有需要编译的源文件
# 我们不仅要编译 TakeOffDemo.cpp，还必须编译 PSDK 的 C++ 封装层

# 5.1 我们自己的源文件
file(GLOB_RECURSE USER_SOURCES "src/*.cpp")

# 5.2 PSDK 封装层的源文件
set(PSDK_WRAPPER_SOURCES
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/application/application.cpp
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/logger/dji_logger.cpp
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/common/osal/osal.cpp
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/hal/hal_uart.cpp
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/hal/hal_network.cpp
    ${PSDK_ROOT_DIR}/samples/sample_c++/platform/linux/manifold2/hal/hal_usb_bulk.cpp
)

# 5.3 定义我们的可执行程序 (TakeOff)
#    (将我们自己的代码和 PSDK 封装代码一起编译)
add_executable(TakeOff 
    ${USER_SOURCES}
    ${PSDK_WRAPPER_SOURCES}
)

# 6. 指定 PSDK 库文件的位置
link_directories(${PSDK_ROOT_DIR}/build/lib)

# 7. 链接我们的程序到 PSDK 库 (TakeOff)
target_link_libraries(TakeOff
    psdk_v3       # PSDK 3.x 核心库
    psdk_utils
    psdk_hal
    pthread       # Linux 线程库
    rt            # Linux 实时库
)

# 8. (可选但推荐) 安装到 bin 目录 (TakeOff)
install(TARGETS TakeOff DESTINATION "bin")