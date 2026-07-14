#!/usr/bin/env bash
set -euo pipefail

GADGET_NAME=pi5
GADGET_DIR=/sys/kernel/config/usb_gadget/${GADGET_NAME}
BRIDGE_NAME=pi5br0
FFS_ROOT=/dev/usb-ffs
BULK_NAMES=(bulk1 bulk2 bulk3)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
STARTUP_BULK=${STARTUP_BULK:-${SCRIPT_DIR}/startup_bulk/startup_bulk}

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root." >&2
    exit 1
fi

if [ -d "${GADGET_DIR}" ] && [ -f "${GADGET_DIR}/UDC" ]; then
    echo "" > "${GADGET_DIR}/UDC" 2>/dev/null || true
fi

pkill -f "${STARTUP_BULK}" 2>/dev/null || true
sleep 0.2

for bulk_name in "${BULK_NAMES[@]}"; do
    ffs_dir="${FFS_ROOT}/${bulk_name}"
    if mountpoint -q "${ffs_dir}"; then
        umount "${ffs_dir}" || true
    fi
done

if ip link show "${BRIDGE_NAME}" >/dev/null 2>&1; then
    ip link set "${BRIDGE_NAME}" down || true
    ip link delete "${BRIDGE_NAME}" type bridge || true
fi

if [ -d "${GADGET_DIR}" ]; then
    rm -f "${GADGET_DIR}/configs/c.1/rndis.usb0" || true
    for bulk_name in "${BULK_NAMES[@]}"; do
        rm -f "${GADGET_DIR}/configs/c.1/ffs.${bulk_name}" || true
    done
    rm -f "${GADGET_DIR}/os_desc/c.1" || true
    rmdir "${GADGET_DIR}/functions/rndis.usb0" 2>/dev/null || true
    for bulk_name in "${BULK_NAMES[@]}"; do
        rmdir "${GADGET_DIR}/functions/ffs.${bulk_name}" 2>/dev/null || true
    done
    rmdir "${GADGET_DIR}/configs/c.1/strings/0x409" 2>/dev/null || true
    rmdir "${GADGET_DIR}/configs/c.1" 2>/dev/null || true
    rmdir "${GADGET_DIR}/strings/0x409" 2>/dev/null || true
    rmdir "${GADGET_DIR}" 2>/dev/null || true
fi

for bulk_name in "${BULK_NAMES[@]}"; do
    rmdir "${FFS_ROOT}/${bulk_name}" 2>/dev/null || true
done
rmdir "${FFS_ROOT}" 2>/dev/null || true

echo "[pi5-gadget] stopped"
