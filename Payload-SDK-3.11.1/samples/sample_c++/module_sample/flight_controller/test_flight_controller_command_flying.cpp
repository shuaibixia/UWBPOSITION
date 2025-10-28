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

    // 设置初始位置信息（任意值即可初始化）
    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;

    // 初始化飞控模块
    ret = DjiFlightController_Init(ridInfo);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Flight controller initialized successfully.");

    // 获取控制权
    ret = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain control authority failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Control authority obtained.");

    sleep(2); // 等待两秒稳定

    // 起飞命令
    ret = DjiFlightController_StartTakeoff();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start takeoff failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Takeoff command sent successfully.");

    // 等待飞行 10 秒后自动降落
    sleep(10);

    ret = DjiFlightController_StartLanding();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Landing failed, errno = 0x%08llX", ret);
        return -1;
    }

    USER_LOG_INFO("Landing command sent successfully.");
    return 0;
}