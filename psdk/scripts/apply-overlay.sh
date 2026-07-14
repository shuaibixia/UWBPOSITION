#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /path/to/Payload-SDK-3.15.0" >&2
    exit 2
fi

sdk_root=$(cd "$1" 2>/dev/null && pwd) || {
    echo "SDK directory does not exist: $1" >&2
    exit 2
}

if [[ ! -f "$sdk_root/CMakeLists.txt" || ! -d "$sdk_root/samples/sample_c" ]]; then
    echo "Not a Payload SDK source directory: $sdk_root" >&2
    exit 2
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
overlay_root=$(cd "$script_dir/../overlay" && pwd)

cp -R "$overlay_root/." "$sdk_root/"

echo "PSDK overlay applied to: $sdk_root"
echo "Application credentials were not changed. Fill dji_sdk_app_info.h locally before building."
