# PSDK 3.15 覆盖层

本目录只保存项目相对 DJI Payload SDK 3.15 官方包的修改，不复制整套第三方 SDK。

## 已包含配置

- Linux 构建目标切换到 Raspberry Pi 示例。
- Pi4 连接模式：`UART + USB BULK`。
- UART：`/dev/ttyUSB0`，CH340 VID/PID `1A86:7523`，921600 baud。
- 新增 `--direction-demo` 终端方向控制。
- Pi5 USB gadget 脚本保存在 `scripts/pi5-usb-gadget/`。

USB BULK 的 `hal_usb_bulk.h` 与 PSDK 3.15 官方树莓派配置一致，因此不在覆盖层重复保存。

## 应用覆盖层

```bash
./psdk/scripts/apply-overlay.sh /path/to/Payload-SDK-3.15.0
```

脚本只复制 `psdk/overlay/` 中的文件，不写入应用凭据。

## 填写本地应用信息

将模板复制到官方 SDK 的 C 和 C++ 示例目录，然后只在本地填写：

```bash
cp psdk/config/dji_sdk_app_info.example.h \
  /path/to/Payload-SDK-3.15.0/samples/sample_c/platform/linux/raspberry_pi/application/dji_sdk_app_info.h

cp psdk/config/dji_sdk_app_info.example.h \
  /path/to/Payload-SDK-3.15.0/samples/sample_c++/platform/linux/raspberry_pi/application/dji_sdk_app_info.h
```

不要把填好后的文件复制回本仓库。

## 编译

在树莓派上执行：

```bash
cmake -S /path/to/Payload-SDK-3.15.0 -B /path/to/Payload-SDK-3.15.0/build
cmake --build /path/to/Payload-SDK-3.15.0/build -j2
```

## 运行方向控制 Demo

```bash
sudo /path/to/Payload-SDK-3.15.0/build/bin/dji_sdk_demo_on_rpi --direction-demo
```

运行顺序：人工或模拟器起飞并悬停，程序连续确认 `IN_AIR`，输入大写 `ENABLE` 后才获取控制权。

```text
w：机头前方 0.5 m/s，1 秒
s：向后 0.5 m/s，1 秒
a：向左 0.5 m/s，1 秒
d：向右 0.5 m/s，1 秒
x：零速度悬停
q：悬停、释放控制权并退出
```

第一次部署或修改控制逻辑后必须先在 DJI 模拟器验证。
