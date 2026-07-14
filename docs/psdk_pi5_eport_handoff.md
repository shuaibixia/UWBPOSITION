# PSDK 树莓派 5 与 Mavic 3E 交接文档

Pi4 前后左右控制 demo 的独立说明：

见 `docs/psdk_pi4_direction_control_demo_cn.md`。

## 当前结论

当前已经验证成功的方案是：

```text
Mavic 3E + 普通 E-Port 开发者套件 + Raspberry Pi 5 + UART-only
```

也就是现在先不依赖 USB BULK，只用串口链路让 PSDK 跑起来。这个方案已经能完成：

- PSDK 初始化
- 识别飞机为 `Mavic 3 Enterprise`
- 读取飞机固件版本
- 进入 DJI 示例程序
- 运行 `[0] Fc subscribe sample`
- 收到飞控订阅数据，包括四元数、GPS、速度

这对当前 UWB 项目是够用的，因为 UWB 定位和基础控制数据量不大。BULK 后面可以继续研究，但它不应该再卡住第一版 UWB 控制原型。

## 当前硬件

```text
飞机：DJI Mavic 3 Enterprise
开发板：普通 E-Port 开发者套件，不是 E-Port V2
计算板：Raspberry Pi 5
SSH：<用户>@<树莓派地址>
PSDK 版本：Payload-SDK-3.15.0
```

E-Port 开发板拨杆按官方普通 E-Port 示例保持：

```text
Host
```

当前 UART 连接方式：

```text
E-Port 开发板 UART TX -> USB-TTL RX
E-Port 开发板 UART RX -> USB-TTL TX
E-Port 开发板 GND     -> USB-TTL GND
USB-TTL 插到树莓派 USB 口
```

USB-TTL 当前识别为 CH340：

```text
/dev/ttyUSB0
VID/PID: 1A86:7523
```

## 当前文件位置

仓库与本地 SDK：

```text
<本地路径>/Payload-SDK-3.15.0
docs/psdk_pi5_eport_handoff.md
```

树莓派 5：

```text
/home/pi5/dji/psdk/Payload-SDK-3.15.0
/home/pi5/dji/psdk/current -> Payload-SDK-3.15.0
/home/pi5/dji/usb-gadget
```

## 当前 PSDK 配置

当前 Pi5 上的工作基线是 `ONLY_UART`：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_UART
```

对应文件：

```text
samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_config.h
samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_config.h
```

UART 配置：

```c
#define LINUX_UART_DEV1 "/dev/ttyUSB0"
#define LINUX_UART_DEV2 "/dev/ttyUSB0"
#define USB_UART_CONNECTED_TO_UAV_VID (0x1A86)
#define USB_UART_CONNECTED_TO_UAV_PID (0x7523)
#define USER_BAUD_RATE "921600"
```

App 信息已经填入 PSDK 示例工程：

```text
samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
```

这个文档不重复写 App Key 和 License，避免到处复制敏感字段。

## 已验证成功的日志

Pi5 的 UART-only 测试已经出现过这些关键信息：

```text
Payload SDK Version : V3.15.0-beta.0-build.2318
Identify AircraftType = Mavic 3 Enterprise, MountPosition = Extension Port
Aircraft version is V17.01.05.08
Start dji sdk application
connect status async sucess
```

进入 `[0] Fc subscribe sample` 后，日志确认收到数据：

```text
opsdk_topic_quaternion: 2078 条
opsdk_topic_gps_position: 39 条
opsdk_topic_velocity: 43 条
connect status async sucess: 47 次
```

这说明“树莓派 5 通过 PSDK 读取飞机飞控数据”这条链路已经跑通。

## 仍然存在的问题

UART 链路虽然能用，但日志里还有很多：

```text
protocol frame crc16 error
```

这说明串口通信有噪声，可能原因包括：

- 杜邦线太长或接触不稳
- USB-TTL 模块质量一般
- CH340 在 `921600` 波特率下稳定性一般
- DJI 官方 demo 启动了太多模块，串口负载偏大
- 上一次 demo 没正常退出，飞机端还在残留推送数据

目前它不是阻塞问题，因为已经能订阅到数据。但下一步写 UWB 程序时应该降低负载。

## 下一步应该做什么

下一步不要继续用 DJI 的完整 demo 当主程序。应该新建一个最小 PSDK 程序，只做这些事：

1. 初始化 PSDK core。
2. 只注册 UART。
3. 只订阅 UWB 项目需要的飞控数据。
4. 暂时不要启用 camera、liveview、widget、speaker、upgrade、gimbal、media。
5. 把订阅到的数据用干净格式打印出来，方便后面接 UWB 算法。

推荐先订阅：

```text
quaternion
gps_position
velocity
```

如果只是做定位辅助和控制验证，这几个已经够用。

## 常用命令

连接 Pi5：

```bash
ssh <用户>@<树莓派地址>
```

检查串口：

```bash
ls -l /dev/ttyUSB*
lsusb | grep -Ei "1a86|ch340"
```

检查当前 PSDK 配置：

```bash
cd /home/pi5/dji/psdk/current
grep -n "CONFIG_HARDWARE_CONNECTION" samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_config.h
grep -n "LINUX_UART_DEV\\|USB_UART_CONNECTED_TO_UAV" samples/sample_c/platform/linux/raspberry_pi/hal/hal_uart.h
grep -n "USER_BAUD_RATE" samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
```

编译：

```bash
cd /home/pi5/dji/psdk/current/build
make -j2
```

运行 C 示例：

```bash
cd /home/pi5/dji/psdk/current
sudo ./build/bin/dji_sdk_demo_on_rpi
```

运行后选择：

```text
0
```

表示进入飞控订阅示例。

查看最新日志里的订阅数据：

```bash
cd /home/pi5/dji/psdk/current
latest=$(ls -t Logs/DJI_*.log | head -1)
grep -aEn "opsdk_topic_quaternion|opsdk_topic_gps_position|opsdk_topic_velocity|Start dji sdk application|connect status async" "$latest" | head -100
```

## BULK 排查历史

之前尝试过 USB BULK-only：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_USB_BULK_DEVICE
```

结果：

```text
Pi5 能创建 USB gadget
能枚举 2CA3:F001
能创建 bulk1/bulk2/bulk3
startup_bulk 也能运行
但是 PSDK 无法完成飞机握手
```

也尝试过 UART+BULK：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_UART_AND_USB_BULK_DEVICE
```

结果：

```text
UART 能识别飞机
但 payload negotiate 阶段失败
出现 Payload negotiate error, returnCode = 225
同时伴随大量 crc16 error
```

Pi4B 官方镜像上，UART+BULK 曾经成功过；但 Pi5 上当前最稳定、最适合继续做 UWB 原型的是 UART-only。

## Pi5 USB gadget 配置历史

Pi5 的 USB gadget 底层曾经修好过。关键点是 `/boot/firmware/config.txt` 里需要在 `[pi5]` 段落配置：

```text
dtoverlay=dwc2,dr_mode=peripheral
```

`/boot/firmware/cmdline.txt` 里有：

```text
modules-load=dwc2
```

不要把 `g_ether` 固定写进 cmdline，因为它可能占用 UDC，影响自定义 gadget。

Pi5 的 UDC 名称：

```text
1000480000.usb
```

BULK 脚本在仓库中的位置：

```text
psdk/scripts/pi5-usb-gadget/raspi-usb-device-start.sh
psdk/scripts/pi5-usb-gadget/raspi-usb-device-stop.sh
psdk/scripts/pi5-usb-gadget/startup_bulk/startup_bulk
```

BULK 端点配置按 DJI 文档：

```text
VID/PID: 2CA3:F001
bulk1: interface 0, IN 0x81, OUT 0x01
bulk2: interface 1, IN 0x82, OUT 0x02
bulk3: interface 2, IN 0x83, OUT 0x03
```

但现阶段先不要依赖 BULK。

## 不要再踩的坑

- 不要把普通 E-Port 和 E-Port V2 的说明混在一起直接套。
- 不要在 `/boot/firmware/cmdline.txt` 里添加 `g_ether`。
- 不要再把当前工作目标写成 BULK-only。
- 不要用 `ping 192.168.90.1` 当作第一步判断，DJI 文档说 PSDK 成功运行后才可能 ping 通飞机侧 IP。
- 不要把 App Key 和 License 到处打印或写进交接文档。
- 如果 demo 异常退出，下次测试前最好重启飞机或 E-Port 开发板，清掉飞机端残留订阅状态。

## 当前推荐路线

短期路线：

```text
UART-only 跑通最小 PSDK 程序
接入 UWB 数据
订阅飞机姿态/GPS/速度
先做定位与控制逻辑验证
```

中期路线：

```text
如果 UART 数据量不够，再回头解决 BULK
如果 UART 噪声影响控制，再换更稳定的 USB-TTL 或降低订阅频率
```

目前最重要的是先把 UWB 项目主链路跑起来，不要继续被 BULK 卡住。

## 2026-07-09 Pi4 开发前链路验收

Pi4 当前作为开发主板，远程连接信息应从本地 SSH 配置读取，不写入仓库：

```text
Tailscale IP: <树莓派 Tailscale 地址>
SSH: <用户>@<树莓派地址>
hostname: <树莓派主机名>
```

当前 PSDK 配置：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_UART_AND_USB_BULK_DEVICE
#define LINUX_UART_DEV1 "/dev/ttyUSB0"
#define LINUX_UART_DEV2 "/dev/ttyUSB0"
#define USB_UART_CONNECTED_TO_UAV_VID (0x1A86)
#define USB_UART_CONNECTED_TO_UAV_PID (0x7523)
```

BULK gadget 状态：

```text
VID/PID: 2CA3:F001
UDC: fe980000.usb
/dev/usb-ffs/bulk1/ep0 ep1 ep2 存在
/dev/usb-ffs/bulk2/ep0 ep1 ep2 存在
/dev/usb-ffs/bulk3/ep0 ep1 ep2 存在
startup_bulk 三个进程存在
```

运行 C demo 并输入 `[0] Fc subscribe sample` 后，链路验收通过：

```text
Payload SDK Version : V3.15.0-beta.0-build.2318
Identify mount position type is Extension Port Type
Identify aircraft series is Mavic 3 Series
Identity uart0 baudrate is 921600 bps
Identify AircraftType = Mavic 3 Enterprise
Aircraft version is V17.01.05.08
Start dji sdk application
```

最新日志统计：

```text
USBBULK channel create: 2
Usb bulk port has inited: 1
opsdk_topic_quaternion: 2504
opsdk_topic_gps_position: 53
opsdk_topic_velocity: 52
connect status async sucess: 47
crc16 error: 0
Error: 0
Warn: 1
```

唯一 warning 是没有音频设备：

```text
No audio device found
```

这个 warning 来自 DJI 示例里的 speaker/widget 初始化，不影响 PSDK 初始化、BULK 通道、飞控订阅或后续 UWB 控制开发。

结论：

```text
Pi4 的 PSDK 主链路已经通。
UART 控制/协商链路正常。
BULK 高速通道已创建。
飞控订阅数据已收到。
可以进入下一步 UWB + 控制逻辑开发。
```

## 2026-07-09 Pi4 ONLY_BULK 复测

在 Pi4 官方/工作环境上复测了：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_ONLY_USB_BULK_DEVICE
```

测试前确认 Pi4 的 USB gadget 状态正常：

```text
Gadget: /sys/kernel/config/usb_gadget/pi4
VID/PID: 2CA3:F001
UDC: fe980000.usb
/dev/usb-ffs/bulk1/ep0 ep1 ep2 存在
/dev/usb-ffs/bulk2/ep0 ep1 ep2 存在
/dev/usb-ffs/bulk3/ep0 ep1 ep2 存在
startup_bulk 三个进程存在
```

结果：

```text
程序 60 秒内只打印 Payload SDK Version。
没有识别飞机。
没有进入 Start dji sdk application。
timeout 清理时打印 get aircraft base info error。
```

strace 证据：

```text
PSDK 打开 /dev/usb-ffs/bulk1/ep1
PSDK 打开 /dev/usb-ffs/bulk1/ep2
PSDK 向 bulk1/ep1 写入第一包
PSDK 等待 bulk1/ep2 回复
飞机/E-Port host 侧没有返回有效数据
```

复测后已经把 Pi4 配置恢复为：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_UART_AND_USB_BULK_DEVICE
```

结论：

```text
官方文档确实提供 ONLY_BULK 配置项和 BULK 端点配置。
但在当前 Mavic 3E + 普通 E-Port 开发者套件 + Pi4 环境下，ONLY_BULK 不能完成 PSDK 握手。
当前可用路线仍然是 UART-only 或 UART+BULK，不要把项目主线卡在 ONLY_BULK 上。
```

同日恢复 `UART+BULK` 后再次测试：

```c
#define CONFIG_HARDWARE_CONNECTION DJI_USE_UART_AND_USB_BULK_DEVICE
```

结果成功：

```text
Identify mount position type is Extension Port Type
Identify aircraft series is Mavic 3 Series
Identity uart0 baudrate is 921600 bps
Payload negotiate has finished
Identify AircraftType = Mavic 3 Enterprise
Aircraft version is V17.01.05.08
Start dji sdk application
```

日志中明确出现 BULK 通道创建：

```text
USBBULK channel create, ifhost 0, vid:pid 2ca3:f001, itf 0 epin 129 epout 1, channel id 1
USBBULK channel create, ifhost 0, vid:pid 2ca3:f001, itf 0 epin 129 epout 1, channel id 2
Usb bulk port has inited, interfaceNum:0.
bulk_recv_task
```

结论：

```text
Pi4 上 UART+BULK 是可用的。
UART 负责识别、协商、控制链路。
BULK 在 UART 协商成功后被 PSDK 创建为高速数据通道。
ONLY_BULK 失败不等于 BULK 硬件或端点不可用，而是当前飞机/E-Port 组合不能只靠 BULK 完成 PSDK 握手。
```
