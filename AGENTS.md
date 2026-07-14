# AI/开发交接规则

## 当前技术基线

- 飞机：DJI Mavic 3 Enterprise。
- PSDK：3.15，Pi4 当前稳定模式为 `DJI_USE_UART_AND_USB_BULK_DEVICE`。
- UART：`/dev/ttyUSB0`，CH340 `1A86:7523`，921600 baud。
- USB BULK：VID/PID `2CA3:F001`，使用官方 `bulk1/2/3` 端点配置。
- `ONLY_BULK` 在 Pi4/Pi5 均未独立完成握手，不要默认切换。
- UWB：Humatics/Time Domain P4xx，历史代码使用 UDP 21210。

## 仓库约束

- 不提交整套 Payload SDK、SDK 压缩包、`build/`、日志或二进制。
- PSDK 自研修改只维护在 `psdk/overlay/`；官方源码由使用者单独下载。
- 不提交真实 `dji_sdk_app_info.h`、App Key、License、账号或 SSH 信息。
- 中文文档是主要交接材料，路径和命令使用占位符，不写个人 IP 或绝对目录。

## 控制安全

- `--direction-demo` 默认速度为 0.5 m/s，每条方向命令持续 1 秒后归零。
- 未确认 `IN_AIR` 和未输入大写 `ENABLE` 时不得获取控制权。
- 异常、终端断开或退出时必须先发送零速度，再释放 joystick 控制权。
- 未经现场操作员明确确认，不执行真实起飞、降落或持续运动测试。

## 下一开发阶段

1. 清理 Pi `eth0` 的历史静态地址冲突并确认 UWB 实际 IP。
2. 读取 P4xx 配置和 Node ID，验证 UDP 21210 应用层通信。
3. 将 UWB 距离/位置输入独立控制循环；数据超时必须输出零速度。
4. 模拟器通过后再安排低速、短时、可人工接管的实飞验证。
