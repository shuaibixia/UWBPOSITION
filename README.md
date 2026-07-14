# UWBPOSITION

基于 Humatics/Time Domain P4xx UWB、树莓派和 DJI Payload SDK 的无人机定位与控制实验工程。

目标链路：

```text
UWB 固定节点 <-> 机载 UWB <-> 以太网 <-> 树莓派
                                        |
                                        +-> 位置解算与修正控制
                                        |
                                        +-> DJI PSDK <-> Mavic 3 Enterprise
```

## 当前状态

- Pi4 + Mavic 3 Enterprise + PSDK 3.15 的 `UART + USB BULK` 链路已验证。
- `--direction-demo` 已在 DJI 模拟器验证前后左右速度控制、悬停和控制权释放。
- `ONLY_BULK` 尚不能独立完成飞机握手，不作为当前主链路。
- UWB 以太网物理链路和 DHCP 已建立；真实 UWB API、节点发现与测距仍需继续验证。

## 目录

| 路径 | 内容 |
|---|---|
| `linuxcodes/` | 旧版 UWB UDP/RCM C++ 原型及 Humatics 接口代码 |
| `uwb_position/` | Python 三维定位算法包 |
| `examples/` | 可直接运行的定位演示 |
| `config/` | UWB 固定节点坐标示例 |
| `psdk/` | PSDK 3.15 最小覆盖层、配置模板和树莓派脚本 |
| `docs/` | Pi4/Pi5、E-Port、方向控制和带宽评估中文文档 |
| `legacy/` | 仅供追溯的旧实验代码，不作为当前运行入口 |

仓库不再提交整套 DJI PSDK、压缩包、编译产物或真实应用凭据。DJI PSDK 应从官方渠道下载，再应用 `psdk/overlay/`。

## Python 定位演示

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m examples.positioning_demo
```

当前坐标和距离单位均为毫米。

运行单元测试：

```bash
python -m unittest discover -s tests
```

## UWB C++ 原型

```bash
cmake -S linuxcodes -B linuxcodes/build
cmake --build linuxcodes/build -j2
```

该原型仍含历史目标 IP 和节点 ID，运行前先阅读 [linuxcodes/README.md](linuxcodes/README.md)。

## PSDK 方向控制

下载并解压官方 PSDK 3.15 后：

```bash
./psdk/scripts/apply-overlay.sh /path/to/Payload-SDK-3.15.0
```

随后按 [psdk/README.md](psdk/README.md) 填写本地应用信息、编译并部署到 Pi。

## 安全与凭据

- 不要提交 DJI App Key、License、账号、SSH 私钥或真实 `.env`。
- 旧 Git 历史曾包含 DJI 应用凭据；相关凭据应在 DJI 开发者后台轮换。
- 飞行控制测试必须先在模拟器验证；真实飞行需要现场操作员持有遥控器并明确确认。

## 文档入口

- [Pi4 方向控制 Demo](docs/psdk_pi4_direction_control_demo_cn.md)
- [Pi5 移植手册](docs/psdk_raspberry_pi5_porting_guide_cn.md)
- [E-Port/PSDK 交接记录](docs/psdk_pi5_eport_handoff.md)
- [UWB 数据带宽评估](docs/uwb_uart_bandwidth_assessment_cn.md)
