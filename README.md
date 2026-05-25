# Climber Vision 2026

RoboMaster 自主瞄准视觉仓库，基于 C++17 和 CMake，当前集成：

- `auto_aim` 自瞄识别、跟踪、解算、射击与 Planner
- `auto_buff` 能量机关识别与解算
- `omniperception` 多相机感知
- `hero lobshot` 吊射图传链路

整体采用四层结构：

- `src/` 应用入口
- `tasks/` 任务模块
- `io/` 相机与串口通信
- `tools/` 日志、数学、YAML、线程安全队列等通用工具

主要作为 2026 赛季的上场保底代码和学习平台，参考了同济 2025 开源视觉框架并按本队需求重构。

## 1. 依赖安装

本仓库当前不依赖 Ceres。OpenVINO 请先按官方文档安装：

- OpenVINO 2025 安装文档：
  [https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-archive-linux.html](https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-archive-linux.html)

Ubuntu 常用依赖如下：

```bash
sudo apt install -y \
    git \
    g++ \
    cmake \
    pkg-config \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    openssh-server \
    screen
```

说明：

- `tasks/lobshot/` 依赖 GStreamer 与 `x265enc`
- 顶层 [CMakeLists.txt](/home/sentry/Desktop/Climber_Vision_2026-main_he/CMakeLists.txt) 默认将 `OpenVINO_DIR` 写为 `/opt/intel/openvino_2025.3.0/runtime/cmake/`
- 如果你的 OpenVINO 安装路径不同，需要自行修改该变量或通过 CMake 传参覆盖

## 2. 编译

完整编译：

```bash
cmake -B build
make -C build/ -j"$(nproc)"
```

编译单个目标：

```bash
make -C build/ hero -j4
make -C build/ auto_aim_test -j4
make -C build/ calibrate_handeye -j4
```

## 3. 常用运行方式

自瞄测试：

```bash
./build/auto_aim_test configs/test.yaml
```

Hero 主程序：

```bash
./build/hero configs/hero.yaml
```

如果当前只接了自瞄相机和主串口，可以在 `configs/hero.yaml` 中设置：

```yaml
enable_lobshot: false
```

这样 `hero` 会完全跳过吊射相机和吊射第二串口，只保留自瞄流程。

其他常用测试：

```bash
./build/camera_test
./build/handeye_test
./build/calibrate_handeye
```

## 4. Hero 吊射双串口模式

Hero 的 lobshot 吊射图传功能已经重新并入当前仓库，不再依赖原来的 ROS 发送端。

代码分工：

- `tasks/lobshot/` 负责 ROI 裁切、静态区域简化、拖影、HEVC 编码和分片调度
- `io/lobshot/` 负责第二串口上的 `0x0310` 数据封包与发送
- [src/hero.cpp](/home/sentry/Desktop/Climber_Vision_2026-main_he/src/hero.cpp) 根据第一串口收到的模式切换自瞄与吊射

工作流程：

1. 第一串口仍使用 `configs/hero.yaml` 中的 `com_port`
2. 第二串口使用 `configs/lobshot.yaml` 中的 `lobshot_com_port`
3. `hero.yaml` 中同时配置 `aim_camera` 和 `lobshot_camera`
4. 下位机通过第一串口发送 `mode=2` 时，`hero` 自动切到吊射模式
5. 进入吊射模式后，程序切换到 `lobshot_camera`，打开第二串口，并发送 `0x0310` 图传数据
6. 模式退出 `2` 后，第二串口停止发送并关闭，同时切回 `aim_camera`

当前 `hero` 已支持按角色选择海康相机配置：

```yaml
aim_camera:
  camera_name: "hikrobot"
  exposure_ms: 2
  gain: 12
  vid_pid: "2bdf:0001"
  user_id: "hero_aim"

lobshot_camera:
  camera_name: "hikrobot"
  exposure_ms: 10
  gain: 12
  vid_pid: "2bdf:0001"
  user_id: "hero_lobshot"
```

建议：

- 双海康相机优先通过 `user_id` 或 `serial_number` 区分
- 如果只配置顶层相机参数，`io::Camera` 会回退到旧的单相机写法

只调试吊射发送链路时，可直接使用脚本：

```bash
python3 scripts/lobshot_send_inner_packet.py --port /dev/ttyACM1
python3 scripts/lobshot_receive_inner_packet.py
python3 scripts/mock_gimbal_serial.py
python3 scripts/mock_lobshot_serial.py
```

## 5. 配置说明

推荐优先关注这两个配置文件：

- [configs/hero.yaml](/home/sentry/Desktop/Climber_Vision_2026-main_he/configs/hero.yaml)：主流程、自瞄、Planner、云台串口、双相机
- [configs/lobshot.yaml](/home/sentry/Desktop/Climber_Vision_2026-main_he/configs/lobshot.yaml)：吊射第二串口、编码参数、图像预处理参数

其中：

- `hero.yaml -> com_port` 是主串口
- `lobshot.yaml -> lobshot_com_port` 是吊射第二串口
- `speed_control: true` 时使用下位机回传弹速，否则使用 YAML 中配置的 `bullet_speed`
- `fire_yaw_tolerance` 和 `fire_pitch_tolerance` 控制 Planner 闭环开火窗口

## 6. 相机与标定说明

海康相机：

- 当前支持通过 `vid_pid + user_id` 或 `vid_pid + serial_number` 选择设备
- 若 `aim_camera` / `lobshot_camera` 未配置，会回退读取顶层相机字段

坐标系注意事项：

1. OpenCV 视角下：前方为 `z`，右方为 `x`，下方为 `y`
2. 手眼标定结果已包含云台到相机的旋转关系
3. 仍需在 YAML 中配置 `R_gimbal2imubody`
4. 输出的 `yaw`、`pitch` 以相机坐标系定义的正方向为参考

## 7. USB 相机提示

感知 USB 相机可以用 `v4l2-ctl` 检查与区分：

```bash
v4l2-ctl -d /dev/video0 --list-ctrls
v4l2-ctl -d /dev/video2 --get-ctrl=sharpness
v4l2-ctl -d /dev/video2 --set-ctrl=sharpness=2
v4l2-ctl -d /dev/video4 --set-ctrl=sharpness=3
```

## 参考

深度参考同济 2025 开源视觉项目，基于其工具层进行二次开发，并按本仓库结构重写应用层。

- 同济源码链接：
  [https://github.com/TongjiSuperPower/sp_vision_25.git](https://github.com/TongjiSuperPower/sp_vision_25.git)
