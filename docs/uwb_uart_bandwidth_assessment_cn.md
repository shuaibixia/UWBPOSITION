# UWB 数据使用 UART 是否够用

## 结论

够用。

根据本地 UWB 文档，当前 UWB 设备是 Humatics / Time Domain P4xx 系列，串口接口默认是：

```text
115200 baud, 8 data bits, no parity, 1 stop bit
无硬件流控
```

文档还提到可选波特率：

```text
9.6k
19.2k
38.4k
57.6k
115.2k 默认
230.4k
460.8k
921.26k
```

但厂商不建议随便高于默认值，除非线很短并经过实际可靠性测试。

## 数据量估算

UART 8N1 通常按 10 bit 传 1 byte 粗算：

```text
115200 baud 约等于 11520 byte/s
921600 baud 约等于 92160 byte/s
```

UWB 文档里典型测距频率：

```text
约 8-10 Hz
```

常见 UWB 消息大小：

```text
RCM_SMALL_RANGE_INFO：主体约 14 byte，加串口前后缀约 20 byte
RCM_FULL_RANGE_INFO：几十 byte 级别
LOC_LOCATION_INFO：几十 byte 级别
```

按保守估算，即使每条 UWB 结果按 100 byte，每秒 10 次：

```text
100 byte * 10 Hz = 1000 byte/s
```

这只占 115200 串口有效吞吐的大约 9%。

如果 50 Hz：

```text
100 byte * 50 Hz = 5000 byte/s
```

仍然低于 115200 串口的有效吞吐。

所以对于本项目：

```text
UWB 位置/距离/速度结果 -> 树莓派 -> PSDK 控制逻辑
```

UART 完全够用。

## 什么时候 UART 可能不够

以下情况才可能需要 USB、Ethernet 或其他高速链路：

- 传完整 waveform scan。
- 打开 full scan debug 数据。
- 高频率传大量原始 UWB 包。
- 每个 range request/response 带几百到 1000 byte 用户数据。
- 同时记录大量原始日志并实时转发。

但这些不是当前 UWB 控制无人机的主链路需求。

## 当前推荐架构

推荐把系统分成两条串口：

```text
UWB 模块 -> 树莓派
  使用 UWB 自己的 UART，默认 115200 即可先测试。

树莓派 -> Mavic 3E / E-Port
  使用 PSDK UART，当前配置 921600。
```

树莓派负责：

```text
读取 UWB 距离或位置
融合/滤波
读取飞机姿态/GPS/速度
计算控制量
通过 PSDK 控制飞机
```

不要把 UWB 原始大日志全部透传给飞机。只需要把控制逻辑需要的少量状态量保留在树莓派本地即可。

## 实施建议

第一步：

```text
UWB 串口先用默认 115200。
PSDK 串口保持 921600。
不要同时提高两个串口速率。
```

第二步：

```text
先只解析 RCM_SMALL_RANGE_INFO 或 LOC_LOCATION_INFO。
不要打开 FULL_SCAN_INFO。
不要打开 waveform scan。
```

第三步：

```text
如果 UWB 输出频率需要提高到 50Hz 以上，再实测串口丢包率。
如默认 115200 不够，再考虑 230400。
```

## 对 PSDK BULK 的影响

这个结论支持当前路线：

```text
先用 UART-only PSDK 继续做 UWB 原型。
不要因为 UWB 数据量担心而卡在 USB BULK。
```

UWB 数据量远小于视频流、图像流、点云或大日志。当前项目不需要为了 UWB 数据本身强行打通 BULK。
