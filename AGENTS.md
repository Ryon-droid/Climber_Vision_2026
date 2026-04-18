# AGENTS.md - Climber Vision 2026

## 概述
这是一个用于 RoboMaster 自主瞄准的 C++17 计算机视觉项目。使用 CMake 构建系统，集成了 OpenCV、Eigen、spdlog、yaml-cpp 和 tinympc。

## 最近更新
- 移除了 hero 机器人的 lobshot 吊射功能
- 将所有机器人的通信方式从 CBoard 改为 Gimbal
- 为 hero 机器人添加了 Planner 轨迹规划功能
- 统一了 OpenVINO 路径配置
- 修复了 tinympc 库链接问题

## 构建命令

### 完整构建
```bash
cmake -B build
make -C build/ -j$(nproc)
```

### 构建单个可执行文件
```bash
make -C build/ <目标名称> -j4
```

示例：
```bash
make -C build/ calibrate_handeye -j4
```

### 运行测试
没有正式的单元测试框架。测试以独立可执行文件形式运行：
```bash
./build/auto_aim_test configs/test.yaml
./build/camera_test
./build/handeye_test
./build/calibrate_handeye
```

---

## 代码规范

### 文件组织
- **目录结构**：四层架构
  - `src/` - 应用入口
  - `tasks/` - 任务模块（auto_aim, auto_buff, omniperception）
  - `io/` - 硬件通信（camera, gimbal）
  - `tools/` - 工具（日志、数学、yaml）

- **重要模块**：
  - `tasks/auto_aim/planner/` - 轨迹规划模块，使用 tinympc 进行轨迹优化
  - `io/gimbal/` - 云台控制模块，替代了原来的 CBoard

- **头文件保护**：使用 `#ifndef PATH__FILENAME_HPP` 格式
  ```cpp
  #ifndef AUTO_AIM__DETECTOR_HPP
  #define AUTO_AIM__DETECTOR_HPP
  // ...
  #endif  // AUTO_AIM__DETECTOR_HPP
  ```

### 命名规范
| 类型 | 规范 | 示例 |
|------|------|------|
| 命名空间 | 小写 | `auto_aim`, `io`, `tools` |
| 类 | PascalCase | `Detector`, `Camera`, `Armor` |
| 函数 | PascalCase | `detect()`, `read()` |
| 变量 | snake_case | `bgr_img`, `threshold_` |
| 成员变量 | 尾随下划线 | `threshold_`, `debug_` |
| 常量 | UPPER_SNAKE | `MAX_RETRY_COUNT` |
| 枚举值 | UPPER_SNAKE | `Color::RED`, `ArmorType::SMALL` |

### 格式化
- **缩进**：2 或 4 空格（与现有文件保持一致）
- **大括号**：K&R 风格
  ```cpp
  if (condition) {
    // code
  } else {
    // code
  }
  ```
- **行长度**：保持在约 100 字符以内
- **包含顺序**：标准库 → 外部库 → 项目头文件
  ```cpp
  #include <vector>
  #include <opencv2/opencv.hpp>
  #include "tools/logger.hpp"
  ```

### 类型
- 使用 `std::chrono::steady_clock::time_point` 表示时间戳
- 使用 `cv::Mat` 处理图像，`cv::Point2f`、`cv::Point3f` 表示点
- 使用 `Eigen::Matrix4d`、`Eigen::Vector3d` 处理变换矩阵
- 优先使用 `std::unique_ptr` 和 `std::shared_ptr`，避免原始指针

### 日志
- 使用 `tools::logger()`（封装了 spdlog）
- 日志级别：`trace`、`debug`、`info`、`warn`、`error`
- **格式说明符必须带冒号**：`{:.2f}` 而不是 `{.02f}`
  ```cpp
  tools::logger()->info("检测到 {} 个装甲板", armors.size());
  tools::logger()->debug("偏航角: {:.4f}, 俯仰角: {:.4f}", yaw, pitch);
  ```

### 异常处理
- 使用 try-catch 块保证异常安全
- 尽量捕获具体异常类型
  ```cpp
  try {
    auto yaml = YAML::LoadFile(config_path);
  } catch (const YAML::BadFile & e) {
    tools::logger()->error("配置文件未找到: {}", e.what());
  } catch (const std::exception & e) {
    tools::logger()->error("错误: {}", e.what());
  }
  ```
- 可能失败的函数返回 bool
- 使用 RAII 管理资源（文件、套接字、相机）

### 配置
- 使用 `configs/` 目录下的 YAML 文件
- 使用 yaml-cpp 加载：
  ```cpp
  auto yaml = YAML::LoadFile(config_path);
  threshold_ = yaml["threshold"].as<double>();
  ```

### 线程安全
- 使用 `std::mutex` 和 `std::lock_guard` 保护共享资源
- 可使用 `tools/thread_safe_queue.hpp` 中的线程安全队列

### 注释
- 避免不必要的注释；代码应自解释
- 复杂算法或非显而易见的逻辑需要注释
- 根据项目需要使用中文注释

---

## 关键库
- **OpenCV** - 图像处理、计算机视觉
- **Eigen3** - 线性代数、矩阵、向量
- **spdlog** - 日志（通过 tools::logger 封装）
- **fmt** - 字符串格式化
- **yaml-cpp** - 配置文件
- **nlohmann_json** - JSON 解析
- **OpenVINO** - AI 推理
- **tinympc** - 轨迹优化

---

## 常见开发任务

### 添加新的可执行文件
1. 在 `CMakeLists.txt` 中添加：
   ```cmake
   add_executable(my_target my_source.cpp)
   target_link_libraries(my_target ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
   ```

### 添加新的任务模块
1. 在 `tasks/` 下创建目录
2. 添加 `CMakeLists.txt` 并创建库目标
3. 在主 `CMakeLists.txt` 中包含

### 使用 Planner 模块
1. 在机器人主程序中初始化 Planner：
   ```cpp
   auto_aim::Planner planner(config_path);
   ```

2. 在主循环中使用 Planner 进行轨迹规划：
   ```cpp
   auto plan = planner.plan(target, gimbal_state.bullet_speed);
   gimbal.send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc);
   ```

3. 在配置文件中添加 Planner 参数：
   ```yaml
   planner:
     fire_thresh: 0.0035
     max_yaw_acc: 10
     max_pitch_acc: 10
   ```

### 调试
- 在配置 yaml 中启用调试模式
- 使用 `cv::imshow()` 进行可视化调试
- 查看 `logs/` 目录的运行时日志
