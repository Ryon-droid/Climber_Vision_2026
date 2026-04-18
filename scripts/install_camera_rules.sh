#!/bin/bash
# 安装USB相机udev规则脚本

echo "正在安装USB相机固定设备名规则..."

# 复制规则文件
sudo cp $(dirname $0)/99-camera-names.rules /etc/udev/rules.d/

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "安装完成！"
echo "现在您可以使用以下固定设备名："
echo "  /dev/camera_left  - 左前相机 (原 video0, USB端口 3-4)"
echo "  /dev/camera_right  - 右前相机 (原 video2, USB端口 3-5)"
echo "  /dev/camera_back_right   - 右后相机 (原 video4, USB端口 3-6)"
echo "  /dev/camera_back_left   - 左后相机 (原 video6, USB端口 3-7)"
echo ""
echo "请重新插拔相机或重启系统使规则生效"
echo "验证命令: ls -la /dev/camera_*"
