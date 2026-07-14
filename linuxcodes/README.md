# UWB C++ 原型

该目录保存早期 Linux UWB 通信原型和 Humatics/Time Domain RCM/RangeNet 接口代码。

## 当前行为

- UDP 端口：`21210`。
- `net/net.cpp` 中目标地址仍写死为 `192.168.1.200`。
- 测距请求中的响应节点 ID 仍写死为 `202`。
- 主循环约每 200 ms 发送一次请求并尝试解析 52 字节响应。

这些值属于历史实验环境。当前设备地址和节点 ID 未确认前，不应直接用于飞行控制。

## 构建

依赖 CMake、C/C++ 编译器和 pthread：

```bash
sudo apt-get install build-essential cmake
cmake -S . -B build
cmake --build build -j2
```

生成程序：

```text
build/UWBPro
```

## 后续改造要求

- 将目标 IP、UDP 端口和响应节点 ID 改为命令行或配置文件参数。
- 修正 socket 初始化的错误处理，不能忽略 `bind`、`sendto` 和 `recvfrom` 返回值。
- 按消息类型和长度解析 RCM/RangeNet 响应，不只依赖固定 52 字节长度。
- 增加超时、连续丢包、无效距离和 Node ID 不匹配测试。
