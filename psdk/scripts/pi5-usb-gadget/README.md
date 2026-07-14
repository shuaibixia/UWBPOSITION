# Raspberry Pi 5 USB BULK gadget

该目录保留 Pi5 上验证过的 BULK-only gadget 配置。它能创建 `bulk1/2/3` 和 `2CA3:F001`，但 Pi5 的 PSDK BULK-only 飞机握手尚未成功。

## 构建 startup_bulk

```bash
sudo apt-get install -y build-essential libaio-dev
make -C startup_bulk
```

## 手动测试

确认 `/boot/firmware/config.txt` 和 `/boot/firmware/cmdline.txt` 已按移植文档启用 `dwc2` 后：

```bash
sudo ./raspi-usb-device-start.sh
sudo ./raspi-usb-device-stop.sh
```

脚本通过自身目录寻找 `startup_bulk/startup_bulk`，不依赖用户名或 `/home` 路径。

## systemd 安装

```bash
sudo install -d /opt/psdk-usb-gadget
sudo cp -R . /opt/psdk-usb-gadget/
sudo cp pi5-usb-gadget.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pi5-usb-gadget.service
```

真实启用服务前先手动执行启动和停止脚本，确认不会占用其他 USB gadget 驱动。
