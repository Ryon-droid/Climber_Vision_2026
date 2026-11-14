# Climber Vision 2026

RoboMaster 自主瞄准视觉仓库，基于 C++ 和 CMake。当前仓库以 **Hero（英雄机器人）** 为主要上场目标，同时保留 infantry/sentry 等其他兵种入口。核心能力：

- `auto_aim` 自瞄识别、跟踪、解算、射击判定与 Planner 轨迹规划
- `auto_buff` 能量机关识别与解算
- `omniperception` 多相机感知
- `lobshot` Hero 专属的吊射图传链路

整体采用四层结构：

- `src/` 应用入口（`hero.cpp`、`infantry.cpp`、`sentry.cpp`、`sentry_aim.cpp`）
- `tasks/` 任务模块
- `io/` 相机与串口通信
- `tools/` 日志、数学、YAML、线程安全队列等通用工具

主要作为 2026 赛季 Hero 的上场代码和学习平台，参考了同济 2025 开源视觉框架并按本队需求重构。

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
- 顶层 `CMakeLists.txt`、`tasks/auto_aim/CMakeLists.txt`、`tasks/auto_buff/CMakeLists.txt`、`tasks/omniperception/CMakeLists.txt` 里各有一行 `set(OpenVINO_DIR "...")`，**四处都要和本机实际安装的 OpenVINO 版本号一致**，否则 `cmake` 会在 `find_package(OpenVINO REQUIRED)` 处报错退出。换过 OpenVINO 版本、或者拿到别的机器上编译时，先用 `ls /opt/intel/` 确认实际版本号，四个文件一起改。

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

Hero 主程序：

```bash
./build/hero configs/hero.yaml
```

如果当前只接了自瞄相机和主串口，可以在 `configs/hero.yaml` 中设置：

```yaml
enable_lobshot: false
```

这样 `hero` 会完全跳过吊射相机和吊射第二串口，只保留自瞄流程。

自瞄流水线单独测试（需要接实体相机）：

```bash
./build/auto_aim_test configs/test_aim.yaml
./build/aim_detect_test -c=configs/hero.yaml
```

**不接硬件、用录像回放验证自瞄流水线**（推荐在改完 `tasks/auto_aim/` 代码后先跑一遍）：

```bash
./build/video_replay_test -c=configs/hero.yaml -v=records/某段录像.avi \
    -o=/tmp/replay_out.mp4 -s=/tmp/replay_snapshots
```

- `-c` 配置文件，默认 `configs/hero.yaml`
- `-v` 待回放视频路径（必填）
- `-o` 标注后的输出视频路径（可选）
- `-s` / `-i` 定期保存标注帧截图的目录 / 间隔帧数（可选，默认每 150 帧一张）

这个工具会完整跑一遍 YOLO 检测 → Tracker 跟踪 → Aimer/Shooter（调试用）→ Planner（真正决定云台指令的那条链路），四元数用单位阵、弹速用配置里的默认值模拟，不需要真实云台/IMU 就能验证识别、跟踪、锁定、开火判定是否正常，运行结束会在 `logs/` 下打印 `frames_with_armor` / `frames_tracking` / `frames_fire`（Shooter 判定）/ `frames_planner_fire`（Planner 判定，即实际会不会真的开火）等统计。

其他常用测试：

```bash
./build/camera_test
./build/handeye_test
./build/calibrate_handeye
```

## 4. Hero 控制链路说明

`src/hero.cpp` 里实际上是**两条并行链路**，容易混淆，务必区分清楚：

```text
主线程：相机取图 → yolo.detect() → tracker.track() → target_queue.push(target)
                                          │                        │
                                          ▼                        ▼
                              aimer.aim() + shooter.shoot()   plan_thread（独立线程）
                                    （仅用于调试画面）              │
                                                                    ▼
                                                      planner.plan() → gimbal.send()
                                                      （真正发给云台/触发开火的指令）
```

- `Aimer::aim()` / `Shooter::shoot()` 只在主线程里跑，结果只用来画调试窗口 `hero_debug` 里的瞄准点/开火颜色，**不会**传给云台。
- 真正控制云台转动、判断是否开火的是独立的 `plan_thread`：从 `target_queue` 里取最新 `Target`，调用 `Planner::plan()` 生成 `Plan`（含 MPC 求解出的 yaw/pitch/速度/加速度轨迹），再通过 `gimbal.send()` 发出去。
- 两条链路各自的开火判据完全不同：`Shooter` 用的是"小陀螺时序比例"，`Planner` 用的是"预测弹着点误差是否小于阈值"，两者开火频率可能差异很大（属于设计上的两套独立判据，不是 bug）。改开火逻辑时要先确认改的是哪一条链路。
- `hero.yaml` 里的 `use_traditional` 字段**目前不生效**：`hero.cpp` 不读这个字段，永远走 `yolo.detect()`（默认 `yolov8`），不会用传统颜色分割检测器。

### 前哨站（Outpost）锁定逻辑

`tasks/auto_aim/target.cpp` 里有一套专门给前哨站用的动态锁定算法（`OutpostAssignment` 3×3 代价匹配 + `choose_outpost_primary_slot` 主打击面决策 + 失配后的快速重新锚定），核心思路是"尽量一直锁在同一个装甲面上"，比简单写死某个槽位号更稳。`Target::has_primary_armor_xyza()` / `Target::primary_armor_xyza()` 是对外暴露的取值接口。

## 5. Hero 吊射双串口模式

Hero 的 lobshot 吊射图传功能已经重新并入当前仓库，不再依赖原来的 ROS 发送端。

代码分工：

- `tasks/lobshot/` 负责 ROI 裁切、静态区域简化、拖影、HEVC 编码和分片调度
- `io/lobshot/` 负责第二串口上的 `0x0310` 数据封包与发送
- `src/hero.cpp` 根据第一串口收到的模式切换自瞄与吊射

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
  exposure_ms: 3
  gain: 13
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

## 6. 配置说明

推荐优先关注这两个配置文件：

- `configs/hero.yaml`：主流程、自瞄、Planner、云台串口、双相机
- `configs/lobshot.yaml`：吊射第二串口、编码参数、图像预处理参数

其中：

- `hero.yaml -> com_port` 是主串口，`baudrate` 需要和下位机一致
- `lobshot.yaml -> lobshot_com_port` 是吊射第二串口
- `enemy_color` 决定 `Tracker` 按哪种颜色过滤敌方装甲板（`red` / `blue`），打错边会导致该颜色的机器人全部识别不到
- `speed_control: true` 时使用下位机回传弹速（内部做了固定窗口 + 差值去重的平滑），否则使用 YAML 中配置的 `bullet_speed`
- `robot_type: "hero"` 会让弹道解算走考虑空气阻力的模型（`tools::TrajectoryMethod::HERO_WITH_AIR_RESISTANCE`），其它兵种走无空气阻力模型
- `fire_thresh` / `fire_yaw_tolerance` / `fire_pitch_tolerance` 控制 Planner 的开火判定，`first_tolerance` / `second_tolerance` / `decision_spin` / `fire_time_min_ratio` / `fire_time_max_ratio` 控制的是 Shooter（调试画面）的开火判定，两组参数不通用，改的时候别改错文件

## 7. 相机与标定说明

海康相机：

- 当前支持通过 `vid_pid + user_id` 或 `vid_pid + serial_number` 选择设备
- 若 `aim_camera` / `lobshot_camera` 未配置，会回退读取顶层相机字段

坐标系注意事项：

1. OpenCV 视角下：前方为 `z`，右方为 `x`，下方为 `y`
2. 手眼标定结果已包含云台到相机的旋转关系
3. 仍需在 YAML 中配置 `R_gimbal2imubody`
4. 输出的 `yaw`、`pitch` 以相机坐标系定义的正方向为参考

## 8. USB 相机提示

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
