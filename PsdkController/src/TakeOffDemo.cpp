/*
 * TakeOffDemo.cpp
 * 脱离无人机也能编译
 * 使用 PSDK 3.11.1 C++ 封装层
 */

#include "application.hpp"                 // 官方 C++ 封装核心
#include "dji_flight_controller.hpp"       // 飞控封装
#include "dji_logger.hpp"                  // 日志
#include <memory>
#include <thread>
#include <chrono>

bool WaitOperationCompletion(
    std::shared_ptr<dji::psdk::IOperationSet> operation_set,
    const std::shared_ptr<dji::psdk::IPromise>& promise,
    int timeout_ms
)
{
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
        USER_LOG_ERROR("操作被拒绝");
        return false;
    }

    return promise->GetState() == dji::psdk::kPromiseStateFulfilled;
}

int main(int argc, char **argv)
{
    // 初始化 PSDK Application
    Application application(argc, argv);
    USER_LOG_INFO("PSDK 初始化完成 (脱离无人机)");

    // 创建 OperationSet
    auto operation_set = std::make_shared<dji::psdk::OperationSet>();

    // 获取 FlightController 封装接口
    auto fc_manager = application.GetCore()->GetModuleManager()
                          ->GetModule<dji::psdk::flight_controller::IFlightControllerManager>();
    if (!fc_manager) {
        USER_LOG_ERROR("获取 FlightControllerManager 失败");
        return -1;
    }

    // 尝试获取飞控实例（脱离无人机可模拟为空）
    std::shared_ptr<dji::psdk::flight_controller::IFlightController> fc = nullptr;
    fc = fc_manager->GetAvailableFlightController(dji::psdk::kFlightControllerIndex1);
    if (!fc) {
        USER_LOG_WARN("未检测到无人机，程序仅测试编译运行，无实际飞控控制");
        return 0;  // 脱离无人机时直接退出
    }

    // 获取飞行控制权
    USER_LOG_INFO("尝试获取控制权...");
    auto obtain_promise = fc->ObtainControlAuthority(operation_set);
    if (!WaitOperationCompletion(operation_set, obtain_promise, 30000)) {
        USER_LOG_ERROR("获取控制权失败");
        return -1;
    }
    USER_LOG_INFO("已获取控制权");

    // 发起起飞指令
    USER_LOG_INFO("发送起飞指令...");
    auto takeoff_promise = fc->StartTakeoff(operation_set);
    bool takeoff_success = WaitOperationCompletion(operation_set, takeoff_promise, 45000);
    if (takeoff_success) {
        USER_LOG_INFO("起飞成功");
        std::this_thread::sleep_for(std::chrono::seconds(5)); // 模拟悬停
    } else {
        USER_LOG_ERROR("起飞失败");
    }

    // 释放控制权
    USER_LOG_INFO("释放控制权");
    auto release_promise = fc->ReleaseControlAuthority(operation_set);
    WaitOperationCompletion(operation_set, release_promise, 10000);
    USER_LOG_INFO("程序退出");

    return 0;
}