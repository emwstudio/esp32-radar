#!/bin/bash
# 用法: flash_and_listen.sh [监听秒数]
# 顺序：先烧录（端口空闲），再监听。loop() 里有心跳，不怕错过启动日志。
set -e
export PATH="$HOME/Library/Python/3.9/bin:$PATH"
cd /Users/emwstudio/esp32/esp32-radar
DUR=${1:-30}
LOG=/tmp/esp32-serial.log

pio run -t upload --upload-port /dev/cu.usbmodem3101 2>&1 | tail -4
echo "--- upload done, listen ${DUR}s ---"
python3 /tmp/esp32_listen.py /dev/cu.usbmodem3101 "$DUR" "$LOG" || true
echo "=== serial log ==="
cat "$LOG"
