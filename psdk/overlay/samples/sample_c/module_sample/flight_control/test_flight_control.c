/**
 ********************************************************************
 * @file    test_flight_control.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dji_flight_controller.h"
#include "test_flight_control.h"
#include "dji_fc_subscription.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include <math.h>
#include <widget_interaction_test/test_widget_interaction.h>
#include <dji_aircraft_info.h>
#include "dji_fts.h"
#include <ctype.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
/* Private constants ---------------------------------------------------------*/
#define DIRECTION_CONTROL_SPEED_MPS                 (0.5F)
#define DIRECTION_CONTROL_ACTION_TIME_MS            (1000U)
#define DIRECTION_CONTROL_HOVER_TIME_MS             (1000U)
#define DIRECTION_CONTROL_PERIOD_MS                 (20U)
#define DIRECTION_CONTROL_AIRBORNE_CONFIRM_SAMPLES  (5U)
#define DIRECTION_CONTROL_AIRBORNE_TIMEOUT_SAMPLES  (50U)
#define DIRECTION_CONTROL_INPUT_POLL_MS             (200)

/* Private types -------------------------------------------------------------*/
typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiTestFlightControlDisplayModeStr;

typedef struct {
    pthread_mutex_t mutex;
    T_DjiFlightControllerJoystickCommand command;
    uint32_t commandDeadlineMs;
    uint32_t commandSequence;
    char commandKey;
    bool commandHasDeadline;
    bool motionActive;
    bool stopRequested;
    bool workerFailed;
    T_DjiReturnCode workerError;
} T_DjiTestDirectionControlState;

/* Private values -------------------------------------------------------------*/
static T_DjiOsalHandler *s_osalHandler = NULL;
static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;
static bool s_isFtsCallbackRegistered = false;
static int32_t s_ftsTriggerCount = 0;
static uint8_t s_mission_state_machine = 0;
static volatile sig_atomic_t s_directionControlSignal = 0;

static const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};

/* Private functions declaration ---------------------------------------------*/
static uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);
static T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void);
static T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void);
static T_DjiFcSubscriptionHeightFusion DjiTest_FlightControlGetValueOfHeightFusion(void);
static T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void);
static T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void);
static dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void);
static bool DjiTest_FlightControlMotorStartedCheck(void);
static bool DjiTest_FlightControlTakeOffInAirCheck(void);
static bool DjiTest_FlightControlLandFinishedCheck(void);
static bool DjiTest_FlightControlMonitoredTakeoff(void);
static bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode);
static bool DjiTest_FlightControlMonitoredLanding(void);
static bool DjiTest_FlightControlGoHomeAndConfirmLanding(void);
static T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(T_DjiFcSubscriptionQuaternion quat);
static T_DjiTestFlightControlVector3f
DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(T_DjiFcSubscriptionPositionFused target,
                                                            T_DjiFcSubscriptionPositionFused origin,
                                                            dji_f32_t targetHeight,
                                                            dji_f32_t originHeight);
static T_DjiTestFlightControlVector3f
DjiTest_FlightControlVector3FSub(T_DjiTestFlightControlVector3f vectorA, T_DjiTestFlightControlVector3f vectorB);
static int DjiTest_FlightControlSignOfData(dji_f32_t data);
static void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY);
static dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v);
static T_DjiReturnCode
DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
static bool DjiTest_FlightControlMoveByPositionOffset(T_DjiTestFlightControlVector3f offsetDesired,
                                                      float yawDesiredInDeg,
                                                      float posThresholdInM,
                                                      float yawThresholdInDeg);
static T_DjiReturnCode DjiTest_FlightControlInit(void);
static T_DjiReturnCode DjiTest_FlightControlDeInit(void);
static void DjiTest_FlightControlTakeOffLandingSample(void);
static void DjiTest_FlightControlPositionControlSample(void);
static void DjiTest_FlightControlGoHomeForceLandingSample(void);
static void DjiTest_FlightControlVelocityControlSample(void);
static void DjiTest_FlightControlArrestFlyingSample(void);
static void DjiTest_FlightControlSetGetParamSample(void);
static void DjiTest_FlightControlSetGetPerceptionParamSample(void);
static void DjiTest_FlightControlPassiveTriggerFtsSample(void);
static void DjiTest_FlightControlSlowRotateMotorSample(void);
static T_DjiReturnCode DjiTest_TriggerFtsEventCallback(void);
static T_DjiReturnCode DjiTest_FlightControlSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect);
static void DjiTest_FlightControlSetModeStartMission(void);
static T_DjiReturnCode DjiTest_FlightControlManualDirectionControlSample(void);
static bool DjiTest_FlightControlConfirmAircraftInAir(void);
static void *DjiTest_FlightControlDirectionControlTask(void *argument);
static void DjiTest_FlightControlSetDirectionCommand(T_DjiTestDirectionControlState *state,
                                                     T_DjiFlightControllerJoystickCommand command,
                                                     char commandKey,
                                                     uint32_t durationMs);
static void DjiTest_FlightControlRequestDirectionControlStop(T_DjiTestDirectionControlState *state);
static void DjiTest_FlightControlDirectionControlSignalHandler(int signalNum);
static T_DjiReturnCode DjiTest_FlightControlSendHoverCommand(uint32_t durationMs);

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlRunSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect)
{
    T_DjiReturnCode returnCode;
    T_DjiReturnCode sampleReturnCode;

    USER_LOG_DEBUG("Init flight Control Sample");
    DjiTest_WidgetLogAppend("Init flight Control Sample");

    returnCode = DjiTest_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    sampleReturnCode = DjiTest_FlightControlSample(flightCtrlSampleSelect);

    USER_LOG_DEBUG("Deinit Flight Control Sample");
    DjiTest_WidgetLogAppend("Deinit Flight Control Sample");
    returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit Flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return sampleReturnCode;
}

/* Private functions definition-----------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlInit(void)
{
    T_DjiReturnCode returnCode;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;

    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    } else if (returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
        USER_LOG_WARN("Subscribe topic quaternion duplicate");
    } else {
        USER_LOG_ERROR("Subscribe topic quaternion failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FlightControlDeInit(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFlightController_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void DjiTest_FlightControlTakeOffLandingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control takeoff-landing sample start");
    DjiTest_WidgetLogAppend("Flight control takeoff-landing sample start");
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");
    s_osalHandler->TaskSleepMs(4000);

    USER_LOG_INFO("--> Step 3: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 3: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 4: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 4: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control takeoff-landing sample end");
    DjiTest_WidgetLogAppend("Flight control takeoff-landing sample end");
}

void DjiTest_FlightControlPositionControlSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control move-by-position sample start");
    DjiTest_WidgetLogAppend("Flight control move-by-position sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO("--> Step 3: Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 3: Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0, 6, 6}, 30, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point failed");
        goto out;
    };

    USER_LOG_INFO("--> Step 4: Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point");
    DjiTest_WidgetLogAppend(
        "--> Step 4: Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {6, 0, -3}, -30, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point failed");
        goto out;
    };

    USER_LOG_INFO("--> Step 5: Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 5: Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {-6, -6, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 6: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 7: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 7: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control move-by-position sample end");
    DjiTest_WidgetLogAppend("Flight control move-by-position sample end");
}

void DjiTest_FlightControlGoHomeForceLandingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control go-home-force-landing sample start");
    DjiTest_WidgetLogAppend("Flight control go-home-force-landing sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO("--> Step 3: Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 3: Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0, 0, 30}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 4: Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 4: Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {10, 0, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 5: Set aircraft current position as new home location!");
    DjiTest_WidgetLogAppend("--> Step 5: Set aircraft current position as new home location!");
    returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set aircraft current position as new home location failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 6: Set go home altitude to 50(m)\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Set go home altitude to 50(m)\r\n");
    returnCode = DjiFlightController_SetGoHomeAltitude(50);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }

    /*! get go home altitude */
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    DjiTest_WidgetLogAppend("Current go home altitude is %d m\r\n", goHomeAltitude);

    USER_LOG_INFO("--> Step 7: Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 7: Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {20, 0, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 8: Go home and confirm force landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 8: Go home and confirm force landing\r\n");
    if (!DjiTest_FlightControlGoHomeAndConfirmLanding()) {
        USER_LOG_ERROR("Go home and confirm force landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful go home and confirm force landing\r\n");
    DjiTest_WidgetLogAppend("Successful go home and confirm force landing\r\n");

    USER_LOG_INFO("-> Step 9: Release joystick authority");
    DjiTest_WidgetLogAppend("-> Step 9: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control go-home-force-landing sample end");
    DjiTest_WidgetLogAppend("Flight control go-home-force-landing sample end");
}

void DjiTest_FlightControlVelocityControlSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control move-by-velocity sample start");
    DjiTest_WidgetLogAppend("Flight control move-by-velocity sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO(
        "--> Step 3: Move with north:0(m/s), east:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 3: Move with north:0(m/s), east:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {0, 0, 5.0}, 0, 2000);

    USER_LOG_INFO("--> Step 4: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 4: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 5: Move with north:-1.5(m/s), east:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 5: Move with north:-1.5(m/s), east:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.5, 2, 0}, 20, 2000);

    USER_LOG_INFO("--> Step 6: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 6: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 7: Move with north:3(m/s), east:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    DjiTest_WidgetLogAppend(
        "--> Step 7: Move with north:3(m/s), east:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {3, 0, 0}, 0, 2500);

    USER_LOG_INFO("--> Step 8: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 8: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 9: Move with north:-1.6(m/s), east:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 9: Move with north:-1.6(m/s), east:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.6, -2, 0}, 0, 2200);

    USER_LOG_INFO("--> Step 10: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 10: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 11: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 11: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 12: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 12: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control move-by-velocity sample end");
    DjiTest_WidgetLogAppend("Flight control move-by-velocity sample end");
}

void DjiTest_FlightControlArrestFlyingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control arrest-flying sample start");
    DjiTest_WidgetLogAppend("Flight control arrest-flying sample start");

    USER_LOG_INFO("--> Step 1: Enable arrest-flying");
    DjiTest_WidgetLogAppend("--> Step 1: Enable arrest-flying");
    returnCode = DjiFlightController_ArrestFlying();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Enable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);

    //you can replace with takeoff to test in air.
    USER_LOG_INFO("--> Step 2: Turn on motors\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Turn on motors\r\n");
    returnCode = DjiFlightController_TurnOnMotors();
    if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on motors successfully, but arrest-flying failed");
        s_osalHandler->TaskSleepMs(4000);
        USER_LOG_INFO("--> Step 3: Turn off motors\r\n");
        DjiTest_WidgetLogAppend("--> Step 3: Turn off motors\r\n");
        returnCode = DjiFlightController_TurnOffMotors();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn off motors failed, error code: 0x%08X", returnCode);
        }
        goto out;
    }

    USER_LOG_INFO("Turn on motors failed.Arrest-flying successfully\r\n");
    DjiTest_WidgetLogAppend("Turn on motors failed.Arrest-flying successfully\r\n");
    s_osalHandler->TaskSleepMs(2000);

    USER_LOG_INFO("--> Step 3: Disable arrest-flying");
    DjiTest_WidgetLogAppend("--> Step 3: Disable arrest-flying");
    returnCode = DjiFlightController_CancelArrestFlying();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Disable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);

    USER_LOG_INFO("--> Step 4: Turn on motors\r\n");
    DjiTest_WidgetLogAppend("--> Step 4: Turn on motors\r\n");
    returnCode = DjiFlightController_TurnOnMotors();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on motors failed and disable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    } else {
        USER_LOG_INFO("Turn on motors successfully and disable arrest-flying successfully\r\n");
        s_osalHandler->TaskSleepMs(4000);
        USER_LOG_INFO("--> Step 5: Turn off motors");
        DjiTest_WidgetLogAppend("--> Step 5: Turn off motors");
        returnCode = DjiFlightController_TurnOffMotors();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn off motors failed, error code: 0x%08X", returnCode);
        }
    }

out:
    USER_LOG_INFO("Flight control arrest-flying sample end");
    DjiTest_WidgetLogAppend("Flight control arrest-flying sample end");
}

void DjiTest_FlightControlSetGetParamSample()
{
    T_DjiReturnCode returnCode;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downloadsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    E_DjiFlightControllerRtkPositionEnableStatus rtkEnableStatus;
    E_DjiFlightControllerRCLostAction rcLostAction;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    uint16_t countryCode;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    USER_LOG_INFO("Flight control set-get-param sample start");
    DjiTest_WidgetLogAppend("Flight control set-get-param sample start");

    returnCode = DjiFlightController_GetCountryCode(&countryCode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get aircraft country code error.");
    }
    USER_LOG_INFO("country code: %hd", countryCode);

    /*! Turn on horizontal vision avoid enable */
    USER_LOG_INFO("--> Step 1: Turn on horizontal visual obstacle avoidance");
    DjiTest_WidgetLogAppend("--> Step 1: Turn on horizontal visual obstacle avoidance");
    returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Get horizontal horizontal visual obstacle status\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Get horizontal horizontal visual obstacle status\r\n");
    returnCode = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
        &horizontalVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current horizontal visual obstacle avoidance status is %d\r\n",
                  horizontalVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on horizontal radar avoid enable */
    USER_LOG_INFO("--> Step 3: Turn on horizontal radar obstacle avoidance");
    DjiTest_WidgetLogAppend("--> Step 3: Turn on horizontal radar obstacle avoidance");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
            DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn on horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        };
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 4: Get horizontal radar obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 4: Get horizontal radar obstacle avoidance status\r\n");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
            &horizontalRadarObstacleAvoidanceStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current horizontal radar obstacle avoidance status is %d\r\n",
                      horizontalRadarObstacleAvoidanceStatus);
    }
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards vision avoid enable */
    USER_LOG_INFO("--> Step 5: Turn on upwards visual obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 5: Turn on upwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 6: Get upwards visual obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Get upwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
        &upwardsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current upwards visual obstacle avoidance status is %d\r\n", upwardsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards radar avoid enable */
    USER_LOG_INFO("--> Step 7: Turn on upwards radar obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 7: Turn on upwards radar obstacle avoidance.");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
            DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn on upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 8: Get upwards radar obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 8: Get upwards radar obstacle avoidance status\r\n");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
            &upwardsRadarObstacleAvoidanceStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current upwards radar obstacle avoidance status is %d\r\n", upwardsRadarObstacleAvoidanceStatus);
        s_osalHandler->TaskSleepMs(1000);
    }

    /*! Turn on downwards vision avoid enable */
    USER_LOG_INFO("--> Step 9: Turn on downwards visual obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 9: Turn on downwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 10: Get downwards visual obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 10: Get downwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
        &downloadsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current downwards visual obstacle avoidance status is %d\r\n",
                  downloadsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Set new go home altitude */
    USER_LOG_INFO("--> Step 11: Set go home altitude to 50(m)");
    DjiTest_WidgetLogAppend("--> Step 11: Set go home altitude to 50(m)");
    returnCode = DjiFlightController_SetGoHomeAltitude(50);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    /*! get go home altitude */
    USER_LOG_INFO("--> Step 12: Get go home altitude\r\n");
    DjiTest_WidgetLogAppend("--> Step 12: Get go home altitude\r\n");
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    s_osalHandler->TaskSleepMs(2000);

    /*! Set rtk enable */
    USER_LOG_INFO("--> Step 13: Set rtk enable status");
    DjiTest_WidgetLogAppend("--> Step 13: Set rtk enable status");
    returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set rtk enable failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 14: Get rtk enable status\r\n");
    DjiTest_WidgetLogAppend("--> Step 14: Get rtk enable status\r\n");
    returnCode = DjiFlightController_GetRtkPositionEnableStatus(&rtkEnableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get rtk enable failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current rtk enable status is %d\r\n", rtkEnableStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Set rc lost action */
    if (aircraftInfoBaseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M300_RTK &&
        aircraftInfoBaseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M350_RTK) {
        USER_LOG_INFO("--> Step 15: Set rc lost action");
        DjiTest_WidgetLogAppend("--> Step 15: Set rc lost action");
        returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set rc lost action failed, error code: 0x%08X", returnCode);
            goto out;
        }
        s_osalHandler->TaskSleepMs(1000);

        USER_LOG_INFO("--> Step 16: Get rc lost action\r\n");
        DjiTest_WidgetLogAppend("--> Step 16: Get rc lost action\r\n");
        returnCode = DjiFlightController_GetRCLostAction(&rcLostAction);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get rc lost action failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current rc lost action is %d\r\n", rcLostAction);
        DjiTest_WidgetLogAppend("Current rc lost action is %d\r\n", rcLostAction);
        s_osalHandler->TaskSleepMs(1000);
    }

out:
    USER_LOG_INFO("Flight control set-get-param sample end");
    DjiTest_WidgetLogAppend("Flight control set-get-param sample end");
}

void DjiTest_FlightControlSetGetPerceptionParamSample()
{
    T_DjiReturnCode returnCode;
    uint16_t exit_reason;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return;
    }

    switch (aircraftInfoBaseInfo.aircraftType)
    {
        case DJI_AIRCRAFT_TYPE_M4T:
        case DJI_AIRCRAFT_TYPE_M4E:
        case DJI_AIRCRAFT_TYPE_M4D:
        case DJI_AIRCRAFT_TYPE_M4TD:
        case DJI_AIRCRAFT_TYPE_M400:
            break;

        default:
            USER_LOG_WARN("aircraft type %d not support", aircraftInfoBaseInfo.aircraftType);
            return;
    }
    USER_LOG_INFO("Flight control set-get-perception-param sample start");
    DjiTest_WidgetLogAppend("Flight control set-get-perception-param sample start");

    /* perception parameter test*/
    USER_LOG_INFO("--> Step 1: DjiFlightController_SetPlanningAlgo");
    DjiTest_WidgetLogAppend("--> Step 1: DjiFlightController_SetPlanningAlgo");
    returnCode = DjiFlightController_SetPlanningAlgo(1);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFlightController_SetPlanningAlgo failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: DjiFlightController_SetMaxVelocity");
    DjiTest_WidgetLogAppend("--> Step 2: DjiFlightController_SetMaxVelocity");
    returnCode = DjiFlightController_SetMaxVelocity(10);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFlightController_SetMaxVelocity failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 3: DjiFlightController_SetMinFlightHeight");
    DjiTest_WidgetLogAppend("--> Step 3: DjiFlightController_SetMinFlightHeight");
    returnCode = DjiFlightController_SetMinFlightHeight(50.0f);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFlightController_SetMinFlightHeight failed, error code: 0x%08X", returnCode);
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 4: DjiFlightController_GetExitReason");
    DjiTest_WidgetLogAppend("--> Step 4: DjiFlightController_GetExitReason");
    returnCode = DjiFlightController_GetExitReason(&exit_reason);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiFlightController_GetExitReason failed, error code: 0x%08X", returnCode);
    };
    USER_LOG_INFO("DjiFlightController_GetExitReason is %d\r\n", exit_reason);
    s_osalHandler->TaskSleepMs(1000);

out:
    USER_LOG_INFO("Flight control set-get-perception-param sample end");
    DjiTest_WidgetLogAppend("Flight control set-get-perception-param perceptionsample end");

}

T_DjiReturnCode DjiTest_FlightControlOpenMisInfoCallback(T_DjiFlightControllerOpenMis eventData)
{
    s_mission_state_machine = eventData.mission_state_machine;
    USER_LOG_INFO("OpenMisInfoCallback state_machine = %d, planning_algo = %d, goal_index = %d, distance_remaining = %f, time_remaining = %f, soe_remaining = %d",
                  eventData.mission_state_machine, eventData.mission_planning_algo, eventData.goal_index,
                  eventData.distance_remaining, eventData.time_remaining, eventData.soe_remaining);
}

T_DjiReturnCode DjiTest_FlightControlCoreTrajCallback(T_DjiFlightControllerCoreTraj eventData)
{
    USER_LOG_INFO("DjiTest_FlightControlCoreTrajCallback code_name = %d, point_num = %d, byte_per_point = %d",
                  eventData.code_name, eventData.point_num, eventData.byte_per_point);
}
void DjiTest_FlightControlSetModeStartMission()
{
    T_DjiReturnCode returnCode;
    T_DjiFlightControllerStartMissionReq req = {0};
    T_DjiFlightControllerStartMissionRsp rsp = {0};
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        return;
    }

    switch (aircraftInfoBaseInfo.aircraftType)
    {
        case DJI_AIRCRAFT_TYPE_M4T:
        case DJI_AIRCRAFT_TYPE_M4E:
        case DJI_AIRCRAFT_TYPE_M4D:
        case DJI_AIRCRAFT_TYPE_M4TD:
        case DJI_AIRCRAFT_TYPE_M400:
            break;

        default:
            USER_LOG_WARN("aircraft type %d not support", aircraftInfoBaseInfo.aircraftType);
            return;
    }

    req.version = 1;
    req.operation = 0;
    req.mea = 10.0f;
    req.fly_vel = 10;
    req.goal_num = 1;
    req.cmd_mode_point_info->lat = 22.578111231;
    req.cmd_mode_point_info->lon = 113.93696;
    req.cmd_mode_point_info->alt = 50.0;

    USER_LOG_INFO("Flight control SetModeStartMission sample start");
    DjiTest_WidgetLogAppend("Flight control SetModeStartMission sample start");

    returnCode = DjiFlightController_SetModeStartMission(req, &rsp);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiTest_FlightControlSetModeStartMission failed, error code: 0x%08X", returnCode);
    } else {
        USER_LOG_INFO("DjiTest_FlightControlSetModeStartMission success ret = %d", returnCode);
    }

    USER_LOG_INFO("Flight control SetModeStartMission sample end");
    DjiTest_WidgetLogAppend("Flight control SetModeStartMission sample end");


    USER_LOG_INFO("Flight control register callback sample start");
    DjiTest_WidgetLogAppend("Flight control register callback sample start");

    returnCode = DjiFlightController_RegisterOpenMisInfoCallBack(DjiTest_FlightControlOpenMisInfoCallback);
    USER_LOG_INFO("RegisterOpenMisInfoCallBack ret = %d", returnCode);
    returnCode = DjiFlightController_RegisterCoreTrajCallBack(DjiTest_FlightControlCoreTrajCallback);
    USER_LOG_INFO("RegisterCoreTrajCallBack ret = %d", returnCode);

    s_osalHandler->TaskSleepMs(10000);

    while (true) {
        if (s_mission_state_machine == 0) {
            USER_LOG_INFO("Mission is finished.");
            break;
        }
        s_osalHandler->TaskSleepMs(1000);
    }

    DjiFlightController_AntiRegisterCoreTrajCallBack();
    DjiFlightController_AntiRegisterOpenMisInfoCallBack();
    USER_LOG_INFO("Flight control register callback sample end");
    DjiTest_WidgetLogAppend("Flight control register callback sample end");

    return;
}

T_DjiReturnCode DjiTest_TriggerFtsEventCallback(void)
{
    USER_LOG_INFO("Received FTS Trigger event, count = %d.", s_ftsTriggerCount);
    if (s_ftsTriggerCount == 0) {
        USER_LOG_WARN("Note: Simulate a trigger failure scenario and return an error value, this function will be invoked again.");
        s_ftsTriggerCount++;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    } else {
        USER_LOG_WARN("Note: This is an empty implementation, and the FTS signal needs to be triggered by the PWM signal.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
}

void DjiTest_FlightControlPassiveTriggerFtsSample(void)
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control passive trigger FTS sample start.");

    if (s_isFtsCallbackRegistered == false) {
        returnCode = DjiFlightController_RegTriggerFtsEventCallback(DjiTest_TriggerFtsEventCallback);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Register trigger FTS event callback failed.");
            return;
        } else {
            s_isFtsCallbackRegistered = true;
            USER_LOG_INFO("Register trigger FTS event callback successfully."
                          "Please wait for the aircraft to trigger the payload to execute FTS action.");
        }
    } else {
        USER_LOG_WARN("FTS trigger event callback has been registered, no need to register again.");
    }
}

static void DjiTest_FlightControlSlowRotateMotorSample(void)
{
    E_DjiFlightControllerElectronicSpeedControllerStatus escStatus;

    T_DjiReturnCode returnCode = 0;

    DjiTest_WidgetLogAppend("Start rotating.");
    USER_LOG_INFO("Start rotating.");
    returnCode = DjiFlightController_StartSlowRotateMotor();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start slow rotate blade failed, error code is 0x%08X", returnCode);
        return ;
    }

    returnCode = DjiFlightController_GetElectronicSpeedControllerStatus(&escStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to get ESC status, error code is 0x%08X", returnCode);
    }

    DjiTest_WidgetLogAppend("ESC status: %s motor(s) rotate mode",
        escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
        escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
        escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");
        USER_LOG_INFO("ESC status: %s motor(s) rotate mode",
        escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
        escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
        escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");

    for (int32_t i = 0; i < 8; i++) {
        s_osalHandler->TaskSleepMs(1000);

        returnCode = DjiFlightController_GetElectronicSpeedControllerStatus(&escStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Fail to get ESC status, error code is 0x%08X", returnCode);
        }

        DjiTest_WidgetLogAppend("ESC status: %s motor(s) rotate mode",
            escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
            escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
            escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");
            USER_LOG_INFO("ESC status: %s motor(s) rotate mode",
            escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
            escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
            escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");
    }

    DjiTest_WidgetLogAppend("Stop rotating.");
    USER_LOG_INFO("Stop rotating.");
    returnCode = DjiFlightController_StopSlowRotateMotor();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_INFO("Stop slow rotate blade failed, error code is 0x%08X", returnCode);
        return ;
    }

    s_osalHandler->TaskSleepMs(1000);

    returnCode = DjiFlightController_GetElectronicSpeedControllerStatus(&escStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to get ESC status, error code is 0x%08X", returnCode);
    }

    DjiTest_WidgetLogAppend("ESC status: %s motor(s) rotate mode",
        escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
        escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
        escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");
        USER_LOG_INFO("ESC status: %s motor(s) rotate mode",
        escStatus == DJI_FLIGHT_CONTROLLER_NO_MOTOR_IN_SLOW_ROTATE_MODE ? "no" :
        escStatus == DJI_FLIGHT_CONTROLLER_SOME_MOTOR_IN_SLOW_ROTATE_MODE ? "some" :
        escStatus == DJI_FLIGHT_CONTROLLER_ALL_MOTOR_IN_SLOW_ROTATE_MODE ? "all" : "(error)");
}

T_DjiReturnCode DjiTest_FlightControlSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect)
{
    switch (flightCtrlSampleSelect) {
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING: {
            DjiTest_FlightControlTakeOffLandingSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING: {
            DjiTest_FlightControlPositionControlSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING: {
            DjiTest_FlightControlGoHomeForceLandingSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING: {
            DjiTest_FlightControlVelocityControlSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING: {
            DjiTest_FlightControlArrestFlyingSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM: {
            DjiTest_FlightControlSetGetParamSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_FTS_TRIGGER: {
            DjiTest_FlightControlPassiveTriggerFtsSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SLOW_ROTATE_BLADE: {
            DjiTest_FlightControlSlowRotateMotorSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PERCEPTION_PARAM: {
            DjiTest_FlightControlSetGetPerceptionParamSample();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_CMD_START_MISSION: {
            DjiTest_FlightControlSetModeStartMission();
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_MANUAL_DIRECTION_CTRL: {
            return DjiTest_FlightControlManualDirectionControlSample();
        }
        default:
            return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
}

static void DjiTest_FlightControlDirectionControlSignalHandler(int signalNum)
{
    (void) signalNum;
    s_directionControlSignal = 1;
}

static bool DjiTest_FlightControlConfirmAircraftInAir(void)
{
    uint32_t sampleIndex;
    uint32_t consecutiveInAirSamples = 0;

    for (sampleIndex = 0; sampleIndex < DIRECTION_CONTROL_AIRBORNE_TIMEOUT_SAMPLES; ++sampleIndex) {
        if (s_directionControlSignal != 0) {
            return false;
        }

        if (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            consecutiveInAirSamples++;
            if (consecutiveInAirSamples >= DIRECTION_CONTROL_AIRBORNE_CONFIRM_SAMPLES) {
                return true;
            }
        } else {
            consecutiveInAirSamples = 0;
        }

        s_osalHandler->TaskSleepMs(100);
    }

    return false;
}

static void DjiTest_FlightControlSetDirectionCommand(T_DjiTestDirectionControlState *state,
                                                     T_DjiFlightControllerJoystickCommand command,
                                                     char commandKey,
                                                     uint32_t durationMs)
{
    uint32_t currentTimeMs = 0;

    s_osalHandler->GetTimeMs(&currentTimeMs);
    pthread_mutex_lock(&state->mutex);
    state->command = command;
    state->commandKey = commandKey;
    state->commandSequence++;
    state->commandHasDeadline = durationMs > 0;
    state->commandDeadlineMs = currentTimeMs + durationMs;
    state->motionActive = durationMs > 0;
    pthread_mutex_unlock(&state->mutex);
}

static void DjiTest_FlightControlRequestDirectionControlStop(T_DjiTestDirectionControlState *state)
{
    pthread_mutex_lock(&state->mutex);
    state->stopRequested = true;
    pthread_mutex_unlock(&state->mutex);
}

static void *DjiTest_FlightControlDirectionControlTask(void *argument)
{
    T_DjiTestDirectionControlState *state = argument;
    T_DjiFlightControllerJoystickCommand command = {0};
    T_DjiReturnCode returnCode;
    uint32_t currentTimeMs = 0;
    uint32_t statusCheckCounter = 0;
    uint32_t commandSequence = 0;
    uint32_t lastLoggedCommandSequence = 0;
    char commandKey = '\0';
    char completedCommandKey = '\0';
    bool stopRequested;
    bool motionCompleted;

    while (s_directionControlSignal == 0) {
        motionCompleted = false;
        s_osalHandler->GetTimeMs(&currentTimeMs);

        pthread_mutex_lock(&state->mutex);
        if (state->commandHasDeadline &&
            (int32_t) (currentTimeMs - state->commandDeadlineMs) >= 0) {
            completedCommandKey = state->commandKey;
            memset(&state->command, 0, sizeof(state->command));
            state->commandHasDeadline = false;
            motionCompleted = state->motionActive;
            state->motionActive = false;
        }
        command = state->command;
        commandKey = state->commandKey;
        commandSequence = state->commandSequence;
        stopRequested = state->stopRequested;
        pthread_mutex_unlock(&state->mutex);

        if (stopRequested) {
            break;
        }

        if (motionCompleted) {
            USER_LOG_INFO("Direction command '%c' completed; zero velocity hover is active.", completedCommandKey);
        }

        if (++statusCheckCounter >= 10) {
            statusCheckCounter = 0;
            if (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
                pthread_mutex_lock(&state->mutex);
                state->workerFailed = true;
                state->workerError = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                state->stopRequested = true;
                pthread_mutex_unlock(&state->mutex);
                USER_LOG_ERROR("Aircraft is no longer confirmed in air; stopping direction control.");
                break;
            }
        }

        returnCode = DjiFlightController_ExecuteJoystickAction(command);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            pthread_mutex_lock(&state->mutex);
            state->workerFailed = true;
            state->workerError = returnCode;
            state->stopRequested = true;
            pthread_mutex_unlock(&state->mutex);
            USER_LOG_ERROR("Execute joystick action failed, error code: 0x%08X", returnCode);
            break;
        }

        if (commandSequence != lastLoggedCommandSequence) {
            USER_LOG_INFO("Direction command '%c' sent: sequence=%u, x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, "
                          "returnCode=0x%08X.", commandKey, commandSequence, command.x, command.y, command.z,
                          command.yaw, returnCode);
            lastLoggedCommandSequence = commandSequence;
        }

        s_osalHandler->TaskSleepMs(DIRECTION_CONTROL_PERIOD_MS);
    }

    return NULL;
}

static T_DjiReturnCode DjiTest_FlightControlSendHoverCommand(uint32_t durationMs)
{
    T_DjiFlightControllerJoystickCommand hoverCommand = {0};
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiReturnCode currentReturnCode;
    uint32_t elapsedTimeMs;

    for (elapsedTimeMs = 0; elapsedTimeMs < durationMs; elapsedTimeMs += DIRECTION_CONTROL_PERIOD_MS) {
        currentReturnCode = DjiFlightController_ExecuteJoystickAction(hoverCommand);
        if (currentReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            returnCode = currentReturnCode;
        }
        s_osalHandler->TaskSleepMs(DIRECTION_CONTROL_PERIOD_MS);
    }

    return returnCode;
}

static T_DjiReturnCode DjiTest_FlightControlManualDirectionControlSample(void)
{
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiReturnCode cleanupReturnCode;
    T_DjiTestDirectionControlState state = {0};
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    T_DjiFlightControllerJoystickCommand command = {0};
    pthread_t controlThread;
    struct sigaction signalAction = {0};
    struct sigaction oldSigInt = {0};
    struct sigaction oldSigTerm = {0};
    struct sigaction oldSigHup = {0};
    struct pollfd inputPoll = {.fd = STDIN_FILENO, .events = POLLIN};
    char inputLine[64];
    char *inputCursor;
    char inputCommand;
    bool controlAuthorityObtained = false;
    bool controlThreadStarted = false;
    bool normalExit = false;
    bool inputReady;
    bool workerFailed;
    T_DjiReturnCode workerError;
    int pollResult;

    s_directionControlSignal = 0;
    signalAction.sa_handler = DjiTest_FlightControlDirectionControlSignalHandler;
    sigemptyset(&signalAction.sa_mask);
    sigaction(SIGINT, &signalAction, &oldSigInt);
    sigaction(SIGTERM, &signalAction, &oldSigTerm);
    sigaction(SIGHUP, &signalAction, &oldSigHup);

    USER_LOG_INFO("Manual direction control demo started.");
    USER_LOG_INFO("The aircraft must be manually airborne and hovering before control is enabled.");
    USER_LOG_INFO("RC must remain in P mode and the operator must be ready to take over.");

    if (!DjiTest_FlightControlConfirmAircraftInAir()) {
        USER_LOG_ERROR("Aircraft is not confirmed in air. Direction control is disabled.");
        returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        goto cleanup;
    }

    printf("\nType ENABLE to obtain joystick authority: ");
    fflush(stdout);
    if (fgets(inputLine, sizeof(inputLine), stdin) == NULL) {
        USER_LOG_ERROR("Input closed before direction control was enabled.");
        returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        goto cleanup;
    }
    inputLine[strcspn(inputLine, "\r\n")] = '\0';
    if (strcmp(inputLine, "ENABLE") != 0) {
        USER_LOG_WARN("Confirmation did not match ENABLE. No control authority was requested.");
        goto cleanup;
    }

    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto cleanup;
    }
    controlAuthorityObtained = true;
    USER_LOG_INFO("Joystick control authority obtained.");

    DjiFlightController_SetJoystickMode(joystickMode);
    USER_LOG_INFO("Joystick mode set to body-frame velocity control with stable mode enabled.");

    if (pthread_mutex_init(&state.mutex, NULL) != 0) {
        USER_LOG_ERROR("Create direction control mutex failed.");
        returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        goto cleanup;
    }

    if (pthread_create(&controlThread, NULL, DjiTest_FlightControlDirectionControlTask, &state) != 0) {
        USER_LOG_ERROR("Create direction control task failed.");
        returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        pthread_mutex_destroy(&state.mutex);
        goto cleanup;
    }
    controlThreadStarted = true;

    printf("\nCommands: w=forward, s=backward, a=left, d=right, x=hover, q=quit\n");
    printf("Each movement uses 0.5 m/s for 1 second, then automatically hovers.\n");

    while (s_directionControlSignal == 0) {
        pthread_mutex_lock(&state.mutex);
        workerFailed = state.workerFailed;
        workerError = state.workerError;
        pthread_mutex_unlock(&state.mutex);
        if (workerFailed) {
            returnCode = workerError;
            break;
        }

        printf("direction> ");
        fflush(stdout);
        inputReady = false;
        while (s_directionControlSignal == 0 && !inputReady) {
            inputPoll.revents = 0;
            pollResult = poll(&inputPoll, 1, DIRECTION_CONTROL_INPUT_POLL_MS);
            if (pollResult > 0) {
                if ((inputPoll.revents & POLLIN) != 0) {
                    inputReady = true;
                } else if ((inputPoll.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                    USER_LOG_ERROR("Terminal input disconnected; stopping direction control.");
                    returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                    goto stop_control;
                }
            } else if (pollResult < 0 && errno != EINTR) {
                USER_LOG_ERROR("Poll terminal input failed, errno: %d", errno);
                returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
                goto stop_control;
            }

            pthread_mutex_lock(&state.mutex);
            workerFailed = state.workerFailed;
            workerError = state.workerError;
            pthread_mutex_unlock(&state.mutex);
            if (workerFailed) {
                returnCode = workerError;
                goto stop_control;
            }
        }

        if (s_directionControlSignal != 0) {
            returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
            break;
        }

        if (fgets(inputLine, sizeof(inputLine), stdin) == NULL) {
            USER_LOG_ERROR("Terminal input closed; stopping direction control.");
            returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
            break;
        }

        inputCursor = inputLine;
        while (isspace((unsigned char) *inputCursor)) {
            inputCursor++;
        }
        if (*inputCursor == '\0') {
            USER_LOG_WARN("Empty command. Enter one of: w s a d x q.");
            continue;
        }
        inputCommand = (char) tolower((unsigned char) *inputCursor++);
        while (isspace((unsigned char) *inputCursor)) {
            inputCursor++;
        }
        if (*inputCursor != '\0') {
            USER_LOG_WARN("Invalid command. Enter one of: w s a d x q.");
            continue;
        }

        memset(&command, 0, sizeof(command));
        switch (inputCommand) {
            case 'w':
                command.x = DIRECTION_CONTROL_SPEED_MPS;
                USER_LOG_INFO("Command forward: x=+0.5 m/s, duration=1000 ms.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 'w', DIRECTION_CONTROL_ACTION_TIME_MS);
                break;
            case 's':
                command.x = -DIRECTION_CONTROL_SPEED_MPS;
                USER_LOG_INFO("Command backward: x=-0.5 m/s, duration=1000 ms.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 's', DIRECTION_CONTROL_ACTION_TIME_MS);
                break;
            case 'a':
                command.y = -DIRECTION_CONTROL_SPEED_MPS;
                USER_LOG_INFO("Command left: y=-0.5 m/s, duration=1000 ms.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 'a', DIRECTION_CONTROL_ACTION_TIME_MS);
                break;
            case 'd':
                command.y = DIRECTION_CONTROL_SPEED_MPS;
                USER_LOG_INFO("Command right: y=+0.5 m/s, duration=1000 ms.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 'd', DIRECTION_CONTROL_ACTION_TIME_MS);
                break;
            case 'x':
                USER_LOG_INFO("Command hover: zero velocity.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 'x', 0);
                break;
            case 'q':
                USER_LOG_INFO("Quit requested; zero velocity and authority release will follow.");
                DjiTest_FlightControlSetDirectionCommand(&state, command, 'q', 0);
                normalExit = true;
                goto stop_control;
            default:
                USER_LOG_WARN("Invalid command. Enter one of: w s a d x q.");
                break;
        }
    }

stop_control:
    if (controlThreadStarted) {
        DjiTest_FlightControlSetDirectionCommand(&state, (T_DjiFlightControllerJoystickCommand) {0}, 'x', 0);
        DjiTest_FlightControlRequestDirectionControlStop(&state);
        pthread_join(controlThread, NULL);
        controlThreadStarted = false;
        pthread_mutex_destroy(&state.mutex);
        USER_LOG_INFO("Direction control task stopped; final zero velocity hover follows.");
    }

cleanup:
    if (controlAuthorityObtained) {
        cleanupReturnCode = DjiTest_FlightControlSendHoverCommand(DIRECTION_CONTROL_HOVER_TIME_MS);
        if (cleanupReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Final hover command failed, error code: 0x%08X", cleanupReturnCode);
            if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                returnCode = cleanupReturnCode;
            }
        }

        cleanupReturnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
        if (cleanupReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", cleanupReturnCode);
            if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                returnCode = cleanupReturnCode;
            }
        } else {
            USER_LOG_INFO("Joystick control authority released.");
        }
    }

    sigaction(SIGINT, &oldSigInt, NULL);
    sigaction(SIGTERM, &oldSigTerm, NULL);
    sigaction(SIGHUP, &oldSigHup, NULL);

    if (normalExit) {
        USER_LOG_INFO("Manual direction control demo finished normally.");
    } else {
        USER_LOG_INFO("Manual direction control demo finished without flight commands or after a safety stop.");
    }

    return returnCode;
}

uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }

    return i;
}

T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiDataTimestamp flightStatusTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &flightStatus,
                                                      sizeof(T_DjiFcSubscriptionFlightStatus),
                                                      &flightStatusTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", djiStat);
        flightStatus = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", flightStatusTimestamp.millisecond,
                       flightStatusTimestamp.microsecond);
        USER_LOG_DEBUG("Flight status: %d.", flightStatus);
    }

    return flightStatus;
}

T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionDisplaymode displayMode;
    T_DjiDataTimestamp displayModeTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                      (uint8_t *) &displayMode,
                                                      sizeof(T_DjiFcSubscriptionDisplaymode),
                                                      &displayModeTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic display mode error, error code: 0x%08X", djiStat);
        displayMode = 0;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", displayModeTimestamp.millisecond,
                       displayModeTimestamp.microsecond);
        USER_LOG_DEBUG("Display mode : %d.", displayMode);
    }

    return displayMode;
}

T_DjiFcSubscriptionHeightFusion DjiTest_FlightControlGetValueOfHeightFusion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionHeightFusion heightFusion = {0};
    T_DjiDataTimestamp timestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                      (uint8_t *) &heightFusion,
                                                      sizeof(T_DjiFcSubscriptionHeightFusion),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic height fusion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond, timestamp.microsecond);
        USER_LOG_DEBUG("Relative height fusion is %f m", heightFusion);
    }

    return heightFusion;
}

T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    return quaternion;
}

T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionPositionFused positionFused = {0};
    T_DjiDataTimestamp positionFusedTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                      (uint8_t *) &positionFused,
                                                      sizeof(T_DjiFcSubscriptionPositionFused),
                                                      &positionFusedTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic position fused error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", positionFusedTimestamp.millisecond,
                       positionFusedTimestamp.microsecond);
        USER_LOG_DEBUG("PositionFused: %f, %f,%f,%d.", positionFused.latitude, positionFused.longitude,
                       positionFused.altitude, positionFused.visibleSatelliteNumber);
    }

    return positionFused;
}

dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
    dji_f32_t relativeHeight = 0;
    T_DjiDataTimestamp relativeHeightTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                      (uint8_t *) &homePointAltitude,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude of home point error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                      (uint8_t *) &altitudeFused,
                                                      sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude fused error, error code: 0x%08X", djiStat);
        return -1;
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", relativeHeightTimestamp.millisecond,
                       relativeHeightTimestamp.microsecond);
    }

    relativeHeight = altitudeFused - homePointAltitude;

    return relativeHeight;
}

bool DjiTest_FlightControlMotorStartedCheck(void)
{
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
           DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
}

bool DjiTest_FlightControlTakeOffInAirCheck(void)
{
    int stillOnGround = 0;
    int timeoutCycles = 110;

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        s_osalHandler->TaskSleepMs(100);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

bool takeoffFinishedCheck(void)
{
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

bool DjiTest_FlightControlLandFinishedCheck(void)
{
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ||
           DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode)
{
    int actionNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiTest_FlightControlGetValueOfDisplayMode() != mode && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr,
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                           DjiTest_FlightControlGetValueOfDisplayMode())].displayModeStr);
        return false;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                      s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr);
        return true;
    }
}

bool DjiTest_FlightControlMonitoredTakeoff(void)
{
    T_DjiReturnCode djiStat;

    //! Start takeoff
    djiStat = DjiFlightController_StartTakeoff();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to take off failed, error code: 0x%08X", djiStat);
        return false;
    }

    //! Motors start check
    if (!DjiTest_FlightControlMotorStartedCheck()) {
        USER_LOG_ERROR("Takeoff failed. Motors are not spinning.");
        return false;
    } else {
        USER_LOG_INFO("Motors spinning...");
    }
    //! In air check
    if (!DjiTest_FlightControlTakeOffInAirCheck()) {
        USER_LOG_ERROR("Takeoff failed. Aircraft is still on the ground, but the "
                       "motors are spinning");
        return false;
    } else {
        USER_LOG_INFO("Ascending...");
    }
    //! Finished takeoff check
    if (!takeoffFinishedCheck()) {
        USER_LOG_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.");
        return false;
    }

    return true;
}

bool DjiTest_FlightControlMonitoredLanding(void)
{
    T_DjiReturnCode djiStat;
    /*! Step 1: Start landing */
    djiStat = DjiFlightController_StartLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start landing failed, error code: 0x%08X", djiStat);
        return false;
    }

    /*! Step 2: check Landing start*/
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action!");
        return false;
    } else {
        /*! Step 3: check Landing finished*/
        if (!DjiTest_FlightControlLandFinishedCheck()) {
            USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                           "Please connect DJI Assistant.");
            return false;
        }
    }

    return true;
}

bool DjiTest_FlightControlGoHomeAndConfirmLanding(void)
{
    T_DjiReturnCode djiStat;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus enableStatus;

    djiStat = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&enableStatus);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get downwards visual obstacle avoidance enable status error");
    }

    djiStat = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    /*! Step 1: Start go home */
    USER_LOG_INFO("Start go home action");
    djiStat = DjiFlightController_StartGoHome();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start to go home failed, error code: 0x%08X", djiStat);
        return false;;
    }

    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)) {
        return false;
    } else {
        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            s_osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }

    /*! Step 2: Start landing */
    USER_LOG_INFO("Start landing action");
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action");
        return false;
    } else {
        while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
               DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            T_DjiFcSubscriptionHeightFusion heightFusion = DjiTest_FlightControlGetValueOfHeightFusion();
            s_osalHandler->TaskSleepMs(1000);
            if (DJI_AIRCRAFT_TYPE_M3E == aircraftInfoBaseInfo.aircraftType || DJI_AIRCRAFT_TYPE_M3T == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M3D == aircraftInfoBaseInfo.aircraftType || DJI_AIRCRAFT_TYPE_M3TD == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M3TA == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4T == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4TD == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4D == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4E == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4TD == aircraftInfoBaseInfo.aircraftType
                || DJI_AIRCRAFT_TYPE_M4D == aircraftInfoBaseInfo.aircraftType
            ) {
                if ((dji_f64_t) 0.45 < heightFusion && heightFusion < (dji_f64_t) 0.55) {
                    break;
                }
            } else {
                if ((dji_f64_t) 0.65 < heightFusion && heightFusion < (dji_f64_t) 0.75) {
                    break;
                }
            }
        }
    }

    /*! Step 4: Confirm Landing */
    USER_LOG_INFO("Start confirm Landing and avoid ground action");
    djiStat = DjiFlightController_StartConfirmLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", djiStat);
        return false;
    }

    if (enableStatus == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
        if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)) {
            return false;
        } else {
            while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
                   DjiTest_FlightControlGetValueOfDisplayMode() ==
                   DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
                s_osalHandler->TaskSleepMs(1000);
            }
        }
    } else {
        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() ==
               DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
            s_osalHandler->TaskSleepMs(1000);
        }
    }

    /*! Step 5: Landing finished check*/
    if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        return false;
    }

    return true;
}

T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat)
{
    T_DjiTestFlightControlVector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = (dji_f64_t) 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = (dji_f64_t) 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}

T_DjiTestFlightControlVector3f
DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(const T_DjiFcSubscriptionPositionFused target,
                                                            const T_DjiFcSubscriptionPositionFused origin,
                                                            const dji_f32_t targetHeight,
                                                            const dji_f32_t originHeight)
{
    T_DjiTestFlightControlVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;

    return deltaNed;
}

T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(const T_DjiTestFlightControlVector3f vectorA,
                                                                const T_DjiTestFlightControlVector3f vectorB)
{
    T_DjiTestFlightControlVector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
}

int DjiTest_FlightControlSignOfData(dji_f32_t data)
{
    return data < 0 ? -1 : 1;
}

void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY)
{
    if (fabs(*commandX) > speedFactor)
        *commandX = speedFactor * DjiTest_FlightControlSignOfData(*commandX);
    if (fabs(*commandY) > speedFactor)
        *commandY = speedFactor * DjiTest_FlightControlSignOfData(*commandY);
}

dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

bool
DjiTest_FlightControlMoveByPositionOffset(const T_DjiTestFlightControlVector3f offsetDesired, float yawDesiredInDeg,
                                          float posThresholdInM, float yawThresholdInDeg)
{
    int timeoutInMilSec = 20000;
    int controlFreqInHz = 50;  // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;

    //! get origin position and relative height(from home point)of aircraft.
    T_DjiFcSubscriptionPositionFused originGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
    dji_f32_t originHeightBaseHomePoint = DjiTest_FlightControlGetValueOfRelativeHeight();
    if (originHeightBaseHomePoint == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec) {
        T_DjiFcSubscriptionPositionFused currentGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
        T_DjiFcSubscriptionQuaternion currentQuaternion = DjiTest_FlightControlGetValueOfQuaternion();
        dji_f32_t currentHeight = DjiTest_FlightControlGetValueOfRelativeHeight();
        if (originHeightBaseHomePoint == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            return false;
        }

        float yawInRad = DjiTest_FlightControlQuaternionToEulerAngle(currentQuaternion).z;
        //! get the vector between aircraft and origin point.

        T_DjiTestFlightControlVector3f localOffset = DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(
            currentGPSPosition,
            originGPSPosition,
            currentHeight,
            originHeightBaseHomePoint);
        //! get the vector between aircraft and target point.
        T_DjiTestFlightControlVector3f offsetRemaining = DjiTest_FlightControlVector3FSub(offsetDesired, localOffset);

        T_DjiTestFlightControlVector3f positionCommand = offsetRemaining;
        DjiTest_FlightControlHorizCommandLimit(speedFactor, &positionCommand.x, &positionCommand.y);

        T_DjiFlightControllerJoystickCommand joystickCommand = {positionCommand.x, positionCommand.y,
                                                                offsetDesired.z + originHeightBaseHomePoint,
                                                                yawDesiredInDeg};
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        if (DjiTest_FlightControlVectorNorm(offsetRemaining) < posThresholdInM &&
            fabs(yawInRad / s_degToRad - yawDesiredInDeg) < yawThresholdInDeg) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        } else {
            if (withinBoundsCounter != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit) {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
            break;
        }
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt) {
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
        return false;
    }

    return true;
}

void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    osalHandler->GetTimeMs(&originTime);
    osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystickMode);
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(2);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }
}

T_DjiReturnCode
DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                DjiTest_WidgetLogAppend("Msdk request ctrl authority!");
                USER_LOG_INFO("[Event]Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                DjiTest_WidgetLogAppend("Msdk release ctrl authority!");
                USER_LOG_INFO("[Event]Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                DjiTest_WidgetLogAppend("Internal request ctrl authority!");
                USER_LOG_INFO("[Event]Internal request to obtain joystick ctrl authority\r\n");
            } else {
                DjiTest_WidgetLogAppend("Internal release ctrl authority!");
                USER_LOG_INFO("[Event]Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                DjiTest_WidgetLogAppend("request ctrl authority!");
                USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
            } else {
                DjiTest_WidgetLogAppend("release ctrl authority!");
                USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("rc lost! reset ctrl authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("rc P mode! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("rc switching mode! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("rc pausing! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("rc request return! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("low battery return! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("low battery land! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            DjiTest_WidgetLogAppend("sdk lost! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("near boundary! rc get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_DOCK_REQUEST_CHANGE_JOYSTICK_CTRL_AUTH_EVENT :
            DjiTest_WidgetLogAppend("dock request! dock get authority");
            USER_LOG_INFO("[Event]Current joystick ctrl authority is change due to the dock request\r\n");
            break;
        default:
            DjiTest_WidgetLogAppend("Unknown ctrl authority event");
            USER_LOG_INFO("[Event]Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_SetFtsTrigger(E_DjiMountPosition position, const char* desc)
{
    T_DjiReturnCode djiStat;
    T_DjiFtsPwmEscTriggerStatus esc_status;

    djiStat = DjiFts_SelectFtsPwmTrigger(position);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("select fts pwm trigger E-PORT error, error code: 0x%08X", djiStat);
        return djiStat;
    }

    djiStat = DjiFts_GetFtsPwmTriggerStatus(&esc_status);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get pwm trigger status error, error code: 0x%08X", djiStat);
        return djiStat;
    }

    if (esc_status.ESC[0].fts_select != position || esc_status.ESC[1].fts_select != position)
    {
        USER_LOG_ERROR("pwm trigger status incorrect");
        return djiStat;
    }

    if (esc_status.ESC[0].fts_status == DJI_FLIGHT_CONTROLLER_FTS_NOT_TRIGGERD && esc_status.ESC[1].fts_status == DJI_FLIGHT_CONTROLLER_FTS_NOT_TRIGGERD)

    USER_LOG_INFO("Set fts trigger position %s success", desc);

    return djiStat;
}

T_DjiReturnCode DjiTest_FtsPwmTriggerSample(E_DjiMountPosition position, const char* port_name)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiTest_SetFtsTrigger(position, port_name);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Test select %s fts pwm trigger failed", port_name);
        return returnCode;
    }

    return returnCode;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
