#!/bin/bash

echo "========== USB 摄像头检查 =========="
echo ""

echo "1. USB 设备列表："
lsusb | grep -i camera || echo "   未检测到摄像头"
echo ""

echo "2. 视频设备文件："
ls -la /dev/video* 2>/dev/null || echo "   未找到视频设备"
echo ""

echo "3. 相机符号链接："
ls -la /dev/camera* 2>/dev/null || echo "   未创建符号链接"
echo ""

echo "4. 设备详细信息："
for i in 0 1 2 3; do
    if [ -e /dev/video$i ]; then
        echo "  video$i:"
        udevadm info --query=all /dev/video$i 2>/dev/null | grep -E "ID_VENDOR|ID_MODEL|DEVPATH" | sed 's/^/    /'
    fi
done
echo ""

echo "5. 权限检查："
for dev in /dev/video* /dev/camera*; do
    if [ -e "$dev" ]; then
        stat -c "  %n: %a (owner: %U, group: %G)" "$dev"
    fi
done
echo ""

echo "6. OpenCV 摄像头读取测试 (3秒):"
python3 << 'EOF'
import cv2
import sys

for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_V4L)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"  video{i}: ✓ 成功读取帧 ({frame.shape})")
        else:
            print(f"  video{i}: ✗ 无法读取帧")
        cap.release()
    else:
        print(f"  video{i}: ✗ 无法打开设备")

# 测试符号链接
for name in ["camera_back_left", "camera_back_right"]:
    cap = cv2.VideoCapture(f"/dev/{name}", cv2.CAP_V4L)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"  /dev/{name}: ✓ 成功读取帧 ({frame.shape})")
        else:
            print(f"  /dev/{name}: ✗ 无法读取帧")
        cap.release()
    else:
        print(f"  /dev/{name}: ✗ 无法打开设备")
EOF

echo ""
echo "========== 检查完成 =========="
