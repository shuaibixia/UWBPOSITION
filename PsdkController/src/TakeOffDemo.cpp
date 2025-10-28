#include <dji_flight_controller.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <unistd.h>
#include <iostream>

int main() {
    T_DjiReturnCode ret;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if (osalHandler == nullptr) {
        std::cout << "Get OSAL handler failed." << std::endl;
        return -1;
    }

    // 设置初始位置信息就是飞机的机场位置
    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;

    // 初始化飞控模块 必须初始化模块 直接调用sample的示例函数和导入库 不获取这个模块是无法使用飞控功能的
    ret = DjiFlightController_Init(ridInfo);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Flight controller initialized successfully.");

    // 获取虚拟摇杆控制权
    ret = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain control authority failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Control authority obtained.");

    sleep(2); 

    // 
    ret = DjiFlightController_StartTakeoff();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start takeoff failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Takeoff command sent successfully.");

    // 飞行一段时间后降落 这是必要的 非则下一次配置起飞会失败 需要重新初始化
    sleep(10);

    ret = DjiFlightController_StartLanding();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Landing failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Landing command sent successfully.");
    return 0;
}