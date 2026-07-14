# Pi4 PSDK 前后左右控制 Demo

## 当前状态

方向控制 demo 已完成编码、Pi4 原生编译、地面保护测试和 DJI 模拟器方向控制验证。

真实飞机的空中移动测试尚未执行。

## 运行条件

```text
主机：CUBE / Raspberry Pi 4B
SSH：<用户>@<树莓派地址>
PSDK：3.15.0
连接：DJI_USE_UART_AND_USB_BULK_DEVICE
飞机：Mavic 3 Enterprise
```

运行前必须满足：

- 在开阔安全区域测试。
- 操作员全程手持遥控器并准备接管。
- Pilot 2 已允许 PSDK 飞控。
- 遥控器保持 P 模式。
- 先由操作员人工起飞并稳定悬停。

## 启动命令

```bash
ssh <用户>@<树莓派地址>
cd /home/<用户>/dji/psdk/current
sudo ./build/bin/dji_sdk_demo_on_rpi --direction-demo
```

程序会连续确认飞机处于空中。只有确认成功后才会显示：

```text
Type ENABLE to obtain joystick authority:
```

输入：

```text
ENABLE
```

之后程序才会请求 joystick 控制权。

## 控制命令

每个命令输入后需要按 Enter：

```text
w：相对机头向前，+0.5 m/s，持续 1 秒
s：相对机头向后，-0.5 m/s，持续 1 秒
a：相对机头向左，-0.5 m/s，持续 1 秒
d：相对机头向右，+0.5 m/s，持续 1 秒
x：立即切换为零速度悬停
q：零速度悬停 1 秒、释放控制权并退出
```

控制模式：

```text
水平：速度控制
垂直：速度控制，z=0
航向：角速度控制，yaw=0
坐标系：机体坐标 FRU
稳定模式：开启
发送频率：50Hz
```

每个移动命令到达 1 秒后，控制线程会自动持续发送零速度，不依赖终端继续输入。

## 安全行为

以下情况都会停止方向控制：

- 飞机未被连续确认在空中。
- 输入不是完整的 `ENABLE`。
- 输入 `q`。
- 按下 `Ctrl+C`。
- SSH/终端断开。
- PSDK joystick 命令返回错误。
- 控制过程中飞行状态不再是 `IN_AIR`。

获得控制权后的统一退出流程：

```text
停止控制线程
连续发送零速度 1 秒
释放 joystick 控制权
反初始化飞控订阅和 PSDK
```

M3E 不使用只支持 M30 的 emergency-brake 接口。

## 建议首次实飞顺序

飞机人工悬停后，依次输入：

```text
ENABLE
w
x
s
x
a
x
d
x
q
```

每一步确认实际方向正确后，再输入下一条。首次测试不要连续快速输入命令。

## 已完成验证

Pi4 编译结果：

```text
dji_sdk_demo_on_rpi：成功
dji_sdk_demo_on_rpi_cxx：成功
```

飞机在地面时运行方向模式，结果：

```text
PSDK 初始化成功
识别 Mavic 3 Enterprise
ApplicationStart 成功
连续 5 秒未确认 IN_AIR
程序拒绝方向控制并以退出码 1 结束
Joystick control authority obtained：0 次
Direction command sent：0 次
crc16 error：0 次
```

普通模式回归也已通过，不带 `--direction-demo` 时原有 PSDK demo 可以正常启动。

## 日志

日志目录：

```text
/home/<用户>/dji/psdk/current/Logs
```

方向命令日志会记录：

```text
命令字母
命令序号
x/y/z/yaw 数值
PSDK 返回码
动作开始和完成时间
控制权获取和释放结果
```

## 源码与备份

仓库中的自研覆盖层：

```text
psdk/overlay
```

Pi4 源码：

```text
/home/<用户>/dji/psdk/current
```

修改前的 Pi4 备份：

```text
/home/<用户>/dji/psdk/backups/<备份目录>
```

实际空中测试尚未执行，必须由操作员现场确认安全条件后进行。
