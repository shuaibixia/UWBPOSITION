/*
 * TakeOffDemo.cpp
 * 存放在: ~/UwbProject/PsdkController/src/
 *
 * 这是一个使用 PSDK 3.x C++ 封装层 (Application) 的起飞程序。
 * (已修正头文件和 std_cast 拼写错误)
 */

// 1. 包含必要的头文件
#include "application.hpp"
#include "dji_flight_controller.hpp"
#include "dji_logger.hpp"
#include "dji_sdk_app_info.h"
#include <memory>
#include <thread>
#include <chrono>

// 声明我们将在这个文件后面定义的辅助函数
bool WaitOperationCompletion(std::shared_ptr<dji::psdk::IOperationSet> operation_set,
                             const std::shared_ptr<dji::psdk::IPromise>& promise,
                             int timeout_ms);

// --- 程序主入口 ---
int main(int argc, char **argv) {
    
    // 2. 初始化 PSDK Core (通过 Application 类)
    // Application 类的构造函数会处理所有 DjiCore_Init() 和平台设置
    USER_LOG_INFO("正在初始化 PSDK (Application)...");
    Application application(argc, argv);
    USER_LOG_INFO("PSDK 初始化完成。");

    // 3. 创建 OperationSet
    auto operation_set = std::make_shared<dji::psdk::OperationSet>();
    if (!operation_set) {
        USER_LOG_ERROR("创建 OperationSet 失败。");
        return -1;
    }

    // 4. 获取飞行控制器模块 (FC Manager)
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待 PSDK 核心启动
    
    auto core = dji::psdk::Core::Instance();
    auto fc_manager = core->GetModuleManager()->GetModule<dji::psdk::flight_controller::IFlightControllerManager>();
    if (!fc_manager) {
        USER_LOG_ERROR("获取 FlightControllerManager 模块失败。");
        return -1;
    }

    // 5. 等待并获取飞行控制器实例 (fc)
    USER_LOG_INFO("等待 M3E 无人机连接...");
    std::shared_ptr<dji::psdk::flight_controller::IFlightController> fc = nullptr;
    auto start_wait_time = std::chrono::high_resolution_clock::now();
    while (fc == nullptr) {
        fc = fc_manager->GetAvailableFlightController(dji::psdk::kFlightControllerIndex1); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - start_wait_time).count() > 10) {
            USER_LOG_ERROR("等待无人机连接超时 (10 秒)。");
            USER_LOG_ERROR("如果正在无飞机测试，这是正常的。程序将退出。");
            return 0; 
        }
    }
    USER_LOG_INFO("M3E 飞行控制器已连接。");

    // 6. 获取飞行控制权
    USER_LOG_INFO("正在请求飞行控制权...");
    auto obtain_promise = fc->ObtainControlAuthority(operation_set);
    if (!WaitOperationCompletion(operation_set, obtain_promise, 30000)) {
        USER_LOG_ERROR("获取飞行控制权失败。请确保物理遥控器(RC)已开机并连接!");
        return -1;
    }
    USER_LOG_INFO("成功获取飞行控制权。");

    // 7. 调用 StartTakeoff()
    USER_LOG_INFO("=========== 发送起飞指令... ============");
    auto takeoff_promise = fc->StartTakeoff(operation_set);

    // 8. 等待操作完成
    bool takeoff_success = WaitOperationCompletion(operation_set, takeoff_promise, 45000); 

    if (takeoff_success) {
        USER_LOG_INFO("=========== 起飞成功。=========== ");
        USER_LOG_INFO("悬停 5 秒...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        USER_LOG_INFO("悬停结束。");
    } else {
        USER_LOG_ERROR("=========== 起飞失败。=========== ");
    }
    
    // 9. 释放控制权
    USER_LOG_INFO("正在释放飞行控制权...");
    auto release_promise = fc->ReleaseControlAuthority(operation_set);
    WaitOperationCompletion(operation_set, release_promise, 10000); 
    USER_LOG_INFO("控制权已释放。");
    USER_LOG_INFO("程序退出。");

    return 0;
}


// --- 必备的辅助函数 ---
bool WaitOperationCompletion(std::shared_ptr<dji::psdk::IOperationSet> operation_set,
                             const std::shared_ptr<dji::psdk::IPromise>& promise,
                             int timeout_ms) {
    auto start_time = std::chrono::high_resolution_clock::now();
    while (promise->GetState() == dji::psdk::kPromiseStatePending) {
        operation_set->ProcessTask(); 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
            USER_LOG_ERROR("操作超时 (Timeout: %d ms)", timeout_ms);
            return false;
        }
    }
    if (promise->GetState() == dji::psdk::kPromiseStateRejected) {
        // (!!! 关键修正 !!!) "std_cast" 是错的, 应该是 "std::static_pointer_cast"
        auto result = std::static_pointer_cast<dji::psdk::common::CommonError>(promise->GetError());
        USER_LOG_ERROR("操作被拒绝 (Rejected)，错误码: %d", result.GetCode());
        return false;
    }
    return promise->GetState() == dji::psdk::kPromiseStateFulfilled;
}