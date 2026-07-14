#!/usr/bin/env bash
set -euo pipefail

GADGET_NAME=pi5
GADGET_DIR=/sys/kernel/config/usb_gadget/${GADGET_NAME}
CFG=${GADGET_DIR}/configs/c.1
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
STARTUP_BULK=${STARTUP_BULK:-${SCRIPT_DIR}/startup_bulk/startup_bulk}
STOP_SCRIPT=${SCRIPT_DIR}/raspi-usb-device-stop.sh
FFS_ROOT=/dev/usb-ffs
BULK_NAMES=(bulk1 bulk2 bulk3)

log() {
    echo "[pi5-gadget] $*"
}

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root." >&2
    exit 1
fi

if [ ! -x "${STARTUP_BULK}" ]; then
    echo "Missing executable: ${STARTUP_BULK}" >&2
    exit 1
fi

modprobe libcomposite
modprobe g_ether || true
rmmod g_ether || true
sleep 1

if ! mountpoint -q /sys/kernel/config; then
    mount -t configfs none /sys/kernel/config
fi

UDC_DEV="$(ls /sys/class/udc 2>/dev/null | head -n 1 || true)"
if [ -z "${UDC_DEV}" ]; then
    echo "No UDC device found. Check dwc2/peripheral boot config and reboot." >&2
    exit 1
fi

if [ -d "${GADGET_DIR}" ]; then
    log "existing gadget found, stopping it first"
    "${STOP_SCRIPT}" || true
fi

mkdir -p "${GADGET_DIR}"
cd "${GADGET_DIR}"

echo 0x2ca3 > idVendor
echo 0xf001 > idProduct
echo 0x0001 > bcdDevice
echo 0x0200 > bcdUSB
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
if [ -f /proc/device-tree/serial-number ]; then
    tr -d '\000' < /proc/device-tree/serial-number > strings/0x409/serialnumber
else
    echo "psdk-rpi5" > strings/0x409/serialnumber
fi
echo "raspberry" > strings/0x409/manufacturer
echo "PI5" > strings/0x409/product

mkdir -p "${CFG}/strings/0x409"
echo 0x80 > "${CFG}/bmAttributes"
echo 250 > "${CFG}/MaxPower"

cfg_str=""
mkdir -p "${FFS_ROOT}"

for bulk_name in "${BULK_NAMES[@]}"; do
    cfg_str="${cfg_str}+${bulk_name^^}"
    ffs_dir="${FFS_ROOT}/${bulk_name}"
    func="functions/ffs.${bulk_name}"

    mkdir -p "${ffs_dir}"
    mkdir -p "${func}"
    ln -s "${func}" "${CFG}/ffs.${bulk_name}"

    if ! mountpoint -q "${ffs_dir}"; then
        mount -o mode=0777,uid=2000,gid=2000 -t functionfs "${bulk_name}" "${ffs_dir}"
    fi

    nohup "${STARTUP_BULK}" "${ffs_dir}" >>"/tmp/pi5-startup_bulk-${bulk_name}.log" 2>&1 &
    sleep 1
done

echo "${cfg_str:1}" > "${CFG}/strings/0x409/configuration"

udevadm settle -t 5 || true
echo "${UDC_DEV}" > UDC

log "started ${GADGET_NAME} BULK-only on UDC ${UDC_DEV}"
log "VID/PID 2CA3:F001"
for bulk_name in "${BULK_NAMES[@]}"; do
    log "${FFS_ROOT}/${bulk_name}/ep*"
done
