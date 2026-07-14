# PSDK 树莓派 5 移植配置手册

这份文档给后续配置其他树莓派 5 使用。目标是让其他 AI 或开发者读完后，可以把 DJI PSDK 3.15.0 配到新的 Raspberry Pi 5 上，并优先跑通 `UART-only` 链路。

## 一句话结论

当前项目已经验证：

```text
Mavic 3E + 普通 E-Port 开发者套件 + Raspberry Pi 5 + UART-only 可以跑通 PSDK
```

已经成功收到飞控订阅数据：

```text
opsdk_topic_quaternion
opsdk_topic_gps_position
opsdk_topic_velocity
```

所以新树莓派 5 的第一目标不是 BULK，也不是 RNDIS，而是先把 `UART-only` 跑通。

## 本地资料位置

仓库保留 PSDK 自研覆盖层、脚本和文档；官方 SDK 源码单独下载：

```text
psdk/overlay
psdk/scripts/pi5-usb-gadget
docs/psdk_pi5_eport_handoff.md
docs/psdk_raspberry_pi5_porting_guide_cn.md
```

如果其他 AI 接手，先读这两个文档：

```text
docs/psdk_raspberry_pi5_porting_guide_cn.md
docs/psdk_pi5_eport_handoff.md
```

## 推荐目标目录

在新的树莓派 5 上建议统一使用这个目录结构：

```text
/home/<用户名>/dji/psdk/Payload-SDK-3.15.0
/home/<用户名>/dji/psdk/current -> Payload-SDK-3.15.0
/home/<用户名>/dji/usb-gadget
/home/<用户名>/dji/downloads
/home/<用户名>/dji/build
```

示例用户如果叫 `pi5`：

```bash
mkdir -p /home/pi5/dji/psdk /home/pi5/dji/usb-gadget /home/pi5/dji/downloads /home/pi5/dji/build
```

## 硬件连接

当前优先使用 UART：

```text
E-Port 开发板 UART TX -> USB-TTL RX
E-Port 开发板 UART RX -> USB-TTL TX
E-Port 开发板 GND     -> USB-TTL GND
USB-TTL 插到树莓派 USB 口
```

注意：

- USB-TTL 的 VCC 不要接，当前只接 `TX/RX/GND`。
- TX/RX 必须交叉。
- E-Port 开发板拨杆按普通 E-Port 官方示例保持 `Host`。
- 当前测试过的 USB-TTL 是 CH340，识别为 `/dev/ttyUSB0`，VID/PID 是 `1A86:7523`。

检查串口：

```bash
ls -l /dev/ttyUSB*
lsusb | grep -Ei "1a86|ch340|serial"
```

预期类似：

```text
/dev/ttyUSB0
ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

## 安装基础依赖

先安装最小构建依赖：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake make gcc g++ libaio-dev libusb-1.0-0-dev
```

说明：

- `OPUS/FFmpeg/OpenCV` 暂时不是跑通 UART-only 的必要条件。
- 如果 CMake 打印 `Cannot Find OPUS`，不影响 UART 初始化和飞控订阅。
- 后面做视频、相机流、音频相关功能时，再补 OPUS/FFmpeg/OpenCV。

## 放置 PSDK 源码

可以从 Mac 本地复制：

```bash
scp -r /path/to/Payload-SDK-3.15.0 <用户>@<树莓派IP>:/home/<用户>/dji/psdk/
```

然后在树莓派上创建软链接：

```bash
cd /home/<用户>/dji/psdk
ln -sfn Payload-SDK-3.15.0 current
```

检查：

```bash
ls -ld /home/<用户>/dji/psdk/current
```

## 顶层 CMake 配置

确认 PSDK 顶层 `CMakeLists.txt` 编译的是 Raspberry Pi 示例。

文件：

```text
/home/<用户>/dji/psdk/current/CMakeLists.txt
```

在 `USE_SYSTEM_ARCH MATCHES LINUX` 分支里应该是：

```cmake
add_subdirectory(samples/sample_c/platform/linux/raspberry_pi)
add_subdirectory(samples/sample_c++/platform/linux/raspberry_pi)
```

不要还是 `manifold2`，否则会编译错平台。

## 填写 App 信息

需要改两个文件：

```text
samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
```

从仓库的无凭据模板复制，再只在本地 SDK 中填写：

```text
psdk/config/dji_sdk_app_info.example.h
```

必须确认这些字段不是默认占位符：

```c
#define USER_APP_NAME
#define USER_APP_ID
#define USER_APP_KEY
#define USER_APP_LICENSE
#define USER_DEVELOPER_ACCOUNT
#define USER_BAUD_RATE "921600"
```

不要在交接文档或 Git 仓库里写 App Key 和 License。真实信息只保留在部署机器的 `dji_sdk_app_info.h`。

## 配置 UART-only 模式

需要改两个文件：

```text
samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_config.h
samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_config.h
```

设置为：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_UART
```

不要先配置成：

```c
DJI_USE_ONLY_USB_BULK_DEVICE
DJI_USE_UART_AND_USB_BULK_DEVICE
DJI_USE_UART_AND_NETWORK_DEVICE
DJI_USE_ONLY_NETWORK_DEVICE
```

当前已验证的稳定基线是 `DJI_USE_ONLY_UART`。

## 配置 UART 设备

需要改两个文件：

```text
samples/sample_c/platform/linux/raspberry_pi/hal/hal_uart.h
samples/sample_c++/platform/linux/raspberry_pi/hal/hal_uart.h
```

如果 USB-TTL 是 CH340，配置为：

```c
#define LINUX_UART_DEV1 "/dev/ttyUSB0"
#define LINUX_UART_DEV2 "/dev/ttyUSB0"

#define USB_UART_CONNECTED_TO_UAV_VID (0x1A86)
#define USB_UART_CONNECTED_TO_UAV_PID (0x7523)
```

如果换了 USB-TTL，需要用 `lsusb` 查新的 VID/PID。例如：

```bash
lsusb
```

常见情况：

```text
CH340: 1A86:7523
FT232: 0403:6001
CP2102: 10C4:EA60
```

## 编译

在树莓派上执行：

```bash
cd /home/<用户>/dji/psdk/current
rm -rf build
mkdir build
cd build
cmake ../
make -j2
```

预期输出二进制：

```text
/home/<用户>/dji/psdk/current/build/bin/dji_sdk_demo_on_rpi
/home/<用户>/dji/psdk/current/build/bin/dji_sdk_demo_on_rpi_cxx
```

如果编译时看到：

```text
Cannot Find OPUS
```

先不用处理。UART-only 和飞控订阅不依赖它。

## 运行验证

运行 C 示例：

```bash
cd /home/<用户>/dji/psdk/current
sudo ./build/bin/dji_sdk_demo_on_rpi
```

成功启动时应该看到类似：

```text
Payload SDK Version : V3.15.0
Identify mount position type is Extension Port Type
Identify aircraft series is Mavic 3 Series
Identity uart0 baudrate is 921600 bps
Identify AircraftType = Mavic 3 Enterprise
Aircraft version is V17.01.05.08
Start dji sdk application
```

出现菜单后输入：

```text
0
```

表示进入：

```text
Fc subscribe sample
```

## 验证订阅数据

查看最新日志：

```bash
cd /home/<用户>/dji/psdk/current
latest=$(ls -t Logs/DJI_*.log | head -1)
echo "$latest"
grep -aEn "opsdk_topic_quaternion|opsdk_topic_gps_position|opsdk_topic_velocity|Start dji sdk application|connect status async" "$latest" | head -100
```

成功时应该看到：

```text
opsdk_topic_quaternion
opsdk_topic_gps_position
opsdk_topic_velocity
connect status async sucess
```

这就说明 PSDK 已经能通过树莓派读取飞机飞控数据。

## UART 噪声问题

如果看到大量：

```text
protocol frame crc16 error
```

不要立刻判断失败。只要仍然能看到：

```text
Start dji sdk application
opsdk_topic_quaternion
opsdk_topic_gps_position
opsdk_topic_velocity
```

就说明链路是通的，只是串口质量不干净。

处理建议：

- 缩短杜邦线。
- 确认 GND 接牢。
- 换一个更稳定的 USB-TTL。
- 不要用完整 DJI demo 做长期主程序。
- 后续写最小 PSDK 程序，降低订阅频率和模块数量。
- 如果上一次 demo 异常退出，重启飞机或 E-Port 开发板，清掉残留订阅状态。

## 为什么不优先 BULK

当前项目历史测试结论：

```text
Pi5 的 USB gadget 能起来。
2CA3:F001 能配置。
bulk1/bulk2/bulk3 能创建。
startup_bulk 能运行。
但 USB BULK-only 无法完成 PSDK 飞机握手。
```

`UART+BULK` 也测过：

```text
UART 能识别飞机。
但 payload negotiate 阶段失败。
报 Payload negotiate error, returnCode = 225。
同时有大量 crc16 error。
```

所以后续移植时不要先折腾 BULK。先用 UART-only 跑通主链路。

## BULK 资料保留位置

如果以后必须回头做 BULK，先看：

```text
psdk/scripts/pi5-usb-gadget
docs/psdk_pi5_eport_handoff.md
```

Pi5 gadget 关键配置历史：

```text
/boot/firmware/config.txt:
  [pi5]
  dtoverlay=dwc2,dr_mode=peripheral

/boot/firmware/cmdline.txt:
  modules-load=dwc2
```

不要把 `g_ether` 固定加到 cmdline。

BULK 端点按 DJI 文档是：

```text
VID/PID: 2CA3:F001
bulk1: interface 0, IN 0x81, OUT 0x01
bulk2: interface 1, IN 0x82, OUT 0x02
bulk3: interface 2, IN 0x83, OUT 0x03
```

但这是后续优化方向，不是新树莓派移植的第一目标。

## 新树莓派移植检查清单

配置前：

- 确认飞机是 Mavic 3E。
- 确认 E-Port 是普通开发者套件，不是 E-Port V2。
- 确认 E-Port 拨杆是 `Host`。
- 确认 USB-TTL 只接 `TX/RX/GND`。
- 确认树莓派能 SSH 登录。

配置中：

- PSDK 放到 `/home/<用户>/dji/psdk/current`。
- 顶层 CMake 编译 `raspberry_pi` 示例。
- App 信息填入 C 和 C++ 两份 `dji_sdk_app_info.h`。
- `dji_sdk_config.h` 设置 `DJI_USE_ONLY_UART`。
- `hal_uart.h` 设置 `/dev/ttyUSB0` 和正确 VID/PID。
- `USER_BAUD_RATE` 是 `921600`。

验证：

- `cmake ../ && make -j2` 能通过。
- `sudo ./build/bin/dji_sdk_demo_on_rpi` 能看到飞机型号。
- 菜单输入 `0` 后能看到订阅 topic。
- 日志出现 `opsdk_topic_quaternion` 即可认为飞控订阅链路已通。

## 给其他 AI 的注意事项

- 不要一上来改 BULK。
- 不要把当前问题误判为 OpenCV、FFmpeg、OPUS 缺失。
- 不要把 `Cannot Find OPUS` 当成 UART-only 失败原因。
- 不要用 `ping 192.168.90.1` 判断第一阶段是否成功，DJI 文档说 PSDK 成功运行后才可能 ping 通飞机侧 IP。
- 不要把普通 E-Port 和 E-Port V2 的 VID/PID 逻辑混用。
- 不要在公开文档里重复写 App Key 和 License；需要时读本地源码文件。
- 如果串口持续失败，优先查 TX/RX 是否交叉、GND 是否接牢、USB-TTL 的 VID/PID 是否和代码一致。

## 当前推荐开发路线

第一阶段：

```text
UART-only 最小 PSDK 程序
订阅 quaternion / gps_position / velocity
接入 UWB 数据
做定位与控制逻辑验证
```

第二阶段：

```text
减少 UART 噪声
降低订阅频率
换更稳定 USB-TTL
整理自己的最小 demo
```

第三阶段：

```text
如果数据量真的不够，再回头解决 USB BULK
```

当前不要让 BULK 阻塞 UWB 主链路。
