# Climber Vision 2026 算法设计文档

## 1. 功能简介与Pipeline

### 1.1 系统概述

Climber Vision 2026 是为 RoboMaster 比赛设计的自主瞄准计算机视觉系统，采用 C++17 开发，基于 CMake 构建系统。系统集成了 OpenCV、Eigen、spdlog 和 yaml-cpp 等库，实现了装甲板检测、目标跟踪、弹道解算、能量机关识别等核心功能。

### 1.2 系统架构

系统采用四层架构设计：

```
┌─────────────────────────────────────┐
│        应用层 (src/)                 │  主程序入口
├─────────────────────────────────────┤
│        任务层 (tasks/)               │  核心算法模块
│   ├── auto_aim/ 自主瞄准              │
│   ├── auto_buff/ 能量机关             │
│   └── omniperception/ 全向感知        │
├─────────────────────────────────────┤
│        硬件层 (io/)                   │  硬件通信
│   ├── camera/ 相机                    │
│   ├── gimbal/ 云台                    │
│   └── cboard/ 裁判系统                 │
├─────────────────────────────────────┤
│        工具层 (tools/)                │  通用工具
│   ├── 卡尔曼滤波                      │
│   ├── PID控制                        │
│   ├── 弹道解算                        │
│   └── 图像工具                        │
└─────────────────────────────────────┘
```

### 1.3 核心功能模块

#### 1.3.1 装甲板检测 (auto_aim/detector)

- **传统方法**：基于颜色分割和几何特征检测
- **深度学习方法**：基于 YOLO 的目标检测（YOLOv5/YOLOv8/YOLO11）
- 支持红/蓝双方颜色识别
- 装甲板类型识别（大装甲/小装甲）
- 装甲板数字识别（0-5号、哨兵、前哨站、基地）

#### 1.3.2 目标跟踪 (auto_aim/tracker)

- 目标状态机管理（搜索/跟踪/锁定）
- 多目标优先级选择
- 目标丢失后的暂态处理
- 支持扩展卡尔曼滤波预测

#### 1.3.3 位姿解算 (auto_aim/solver)

- 基于 PnP 的单目测距
- 相机-云台-IMU 坐标系转换
- 重投影误差优化
- 支持 yaw 角优化

#### 1.3.4 弹道解算 (tools/trajectory)

- 考虑空气阻力的弹道模型
- 适用于不同枪口的弹道补偿
- 支持 Hero 高抛弹道

#### 1.3.5 能量机关 (auto_buff)

- 扇叶检测与识别
- 小能量机关：匀速旋转模型
- 大能量机关：简谐运动模型
- 扩展卡尔曼滤波预测

### 1.4 数据处理 Pipeline

```
┌──────────┐    ┌───────────┐    ┌──────────┐    ┌───────────┐    ┌─────────┐
│  相机    │───▶│  目标检测 │───▶│  分类器  │───▶│  跟踪器   │───▶│  解算器 │
│  获取    │    │ (YOLO/   │    │ (数字    │    │ (状态机/ │    │ (PnP/   │
│  图像    │    │  传统)   │    │  识别)   │    │  EKF)    │    │  弹道)  │
└──────────┘    └───────────┘    └──────────┘    └───────────┘    └─────────┘
                                                                     │
                                                                     ▼
                                                              ┌─────────────┐
                                                              │   云台控制   │
                                                              │   (PID)     │
                                                              └─────────────┘
```

---

## 2. 重要算法原理阐述与公式推导

### 2.1 装甲板检测算法

#### 2.1.1 传统方法：基于颜色分割与几何特征

**Step 1: 颜色分割**

根据敌方颜色（红/蓝）进行图像二值化：

```cpp
// 红色通道: R - B > threshold
// 蓝色通道: B - R > threshold
cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
```

**Step 2: 轮廓提取与灯条筛选**

使用 `cv::minAreaRect` 获取最小外接矩形，计算灯条几何特征：

- **长宽比**: `ratio = length / width`
- **角度误差**: `angle_error = |angle - 90°|`
- **筛选条件**:
  - `min_lightbar_ratio < ratio < max_lightbar_ratio`
  - `angle_error < max_angle_error`
  - `length > min_lightbar_length`

**Step 3: 装甲板匹配**

两个同颜色灯条组合形成装甲板，计算几何特征：

- **中点连线比**: `ratio = mid_line_length / max_lightbar_length`
- **边长比**: `side_ratio = max_lightbar_length / min_lightbar_length`
- **矩形误差**: `rectangular_error = |angle(lightbars) - 90°|`

#### 2.1.2 深度学习方法：YOLO

系统支持 YOLOv5、YOLOv8、YOLO11 三种模型架构，统一接口设计：

```cpp
class YOLOBase {
public:
    virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
    virtual std::list<Armor> postprocess(...) = 0;
};
```

**模型推理流程**：

1. 图像预处理（Letterbox 缩放）
2. 模型推理获取预测框
3. NMS 后处理
4. 关键点解码（装甲板四个角点）

### 2.2 PCA 角点精修算法

参考同济大学 2024 视觉方案，使用 PCA 回归角点：

**算法原理**：

1. 在灯条 ROI 内提取高亮像素点云
2. 使用 PCA 计算主成分方向（对称轴方向）
3. 沿轴向搜索亮度跳变点作为角点

```cpp
// PCA 计算对称轴方向
cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), 
                 pca.eigenvectors.at<float>(0, 1));
```

### 2.3 PnP 位姿解算算法

#### 2.3.1 问题定义

已知：

- 装甲板在图像上的四个角点坐标 $p_i = (u_i, v_i), i=1,2,3,4$
- 装甲板在世界坐标系下的三维坐标 $P_i = (X_i, Y_i, Z_i)$
- 相机内参矩阵 $K$

求解：

- 装甲板相对于相机的旋转矩阵 $R$ 和平移向量 $t$

#### 2.3.2 DLT + SVD 求解

构建线性方程组：

$$
\begin{bmatrix}
X_1 & Y_1 & Z_1 & 1 & 0 & 0 & 0 & 0 & -u_1X_1 & -u_1Y_1 & -u_1Z_1 \\
0 & 0 & 0 & 0 & X_1 & Y_1 & Z_1 & 1 & -v_1X_1 & -v_1Y_1 & -v_1Z_1 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots
\end{bmatrix}
\begin{bmatrix}
r_1 \\ r_2 \\ r_3 \\ t_x \\ r_4 \\ r_5 \\ r_6 \\ t_y \\ r_7 \\ r_8 \\ r_9
\end{bmatrix}
=
\begin{bmatrix}
u_1 \\ v_1 \\ \vdots
\end{bmatrix}
$$

使用 SVD 求解超定方程组，得到旋转矩阵和平移向量。

#### 2.3.3 坐标转换

```
相机坐标系 ──(手眼标定)──> 云台坐标系 ──(IMU姿态)──> 世界坐标系
```

```cpp
// 相机 -> 云台
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

// 云台 -> 世界
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

### 2.4 扩展卡尔曼滤波 (EKF)

#### 2.4.1 算法原理

**预测步**：

$$
\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_k)
$$

$$
P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k
$$

**更新步**：

$$
y_k = z_k - h(\hat{x}_{k|k-1})
$$

$$
S_k = H_k P_{k|k-1} H_k^T + R_k
$$

$$
K_k = P_{k|k-1} H_k^T S_k^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k y_k
$$

$$
P_{k|k} = (I - K_k H_k) P_{k|k-1}
$$

#### 2.4.2 小能量机关预测模型

**状态向量**：

$$
x = [\theta]^T
$$

其中 $\theta$ 为扇叶角度。

**状态转移方程**（匀速模型）：

$$
\theta_{k+1} = \theta_k + \omega \cdot \Delta t
$$

其中 $\omega = \pi/3$ rad/s 为固定角速度。

#### 2.4.3 大能量机关预测模型

**状态向量**：

$$
x = [\theta, v, a, \omega, \phi]^T
$$

其中：
- $\theta$: 角度
- $v$: 角速度
- $a$: 幅值 ($0.78 \sim 1.045$)
- $\omega$: 周期参数 ($1.884 \sim 2.000$)
- $\phi$: 相位

**状态转移方程**（简谐运动模型）：

$$
\theta(t) = -\frac{a}{\omega}\cos(\omega t + \phi) + \frac{a}{\omega}\cos(\phi) + (2.09 - a)t
$$

### 2.5 弹道解算算法

#### 2.5.1 无空气阻力模型

水平距离 $d$，高度差 $h$：

$$
h = d \tan\alpha - \frac{g d^2}{2 v_0^2 \cos^2\alpha}
$$

解得俯仰角 $\alpha$：

$$
\alpha = \arctan\left( \frac{v_0^2 \pm \sqrt{v_0^4 - g(g d^2 + 2 h v_0^2)}}{g d + h} \right)
$$

#### 2.5.2 考虑空气阻力模型（Hero 专用）

阻力公式：

$$
F_d = \frac{1}{2} \rho v^2 C_d A
$$

其中：
- $\rho$: 空气密度 ($1.169 kg/m^3$)
- $C_d$: 阻力系数
- $A$: 弹丸截面积

使用数值积分（欧拉法）求解微分方程：

```cpp
// 简化模型
double drag = dragCoeff_ * v * v * sign(v);
vx -= drag * dt;
vz -= (g + drag) * dt;
```

### 2.6 PID 控制算法

#### 2.6.1 位置式 PID

$$
u(k) = K_p e(k) + K_i \sum_{i=0}^{k} e(i) + K_d [e(k) - e(k-1)]
$$

**增量形式**（避免积分饱和）：

$$
\Delta u(k) = K_p [e(k) - e(k-1)] + K_i e(k) + K_d [e(k) - 2e(k-1) + e(k-2)]
$$

#### 2.6.2 角度 PID

对于云台控制，需要使用角度 PID 处理角度回绕问题：

```cpp
float error = set - fdb;
if (angular_) {
    if (error > M_PI) error -= 2 * M_PI;
    if (error < -M_PI) error += 2 * M_PI;
}
```

---

## 3. 算法性能分析与优化

### 3.1 装甲板检测性能

| 方法 | 检测速度 (FPS) | 精度 | 鲁棒性 |
|------|---------------|------|--------|
| 传统方法 | 100+ | 中 | 依赖光照 |
| YOLOv5 | ~60 | 高 | 较好 |
| YOLOv8 | ~50 | 很高 | 好 |
| YOLO11 | ~55 | 很高 | 好 |

### 3.2 传统方法优缺点

**优点**：

1. **速度快**：纯 CPU 计算，无需 GPU
2. **可解释性强**：每一步都有明确的物理意义
3. **资源消耗低**：内存占用小

**缺点**：

1. **依赖阈值**：需要手动调整 `threshold_` 参数
2. **光照敏感**：环境变化影响大
3. **误检率高**：复杂背景下容易误检

### 3.3 深度学习方法优缺点

**优点**：

1. **精度高**：mAP 可达 95% 以上
2. **鲁棒性好**：对光照变化不敏感
3. **泛化能力强**：可识别多种目标

**缺点**：

1. **推理速度**：依赖硬件性能
2. **模型更新**：需要重新训练
3. **可解释性差**：难以调试错误案例

### 3.4 优化方案

#### 3.4.1 检测优化

1. **ROI 裁剪**：只处理可能包含目标的图像区域
2. **多尺度推理**：对不同大小目标使用不同输入尺寸
3. **TensorRT 加速**：使用 INT8 量化提升推理速度
4. **模型剪枝**：去除冗余通道

#### 3.4.2 跟踪优化

1. **IoU 匹配**：使用 IoU 匹配目标
2. **卡尔曼滤波**：预测目标位置，减少匹配延迟
3. **状态机优化**：增加暂态处理，减少目标切换

#### 3.4.3 解算优化

1. **重投影优化**：迭代优化 yaw 角
2. **多线程处理**：并行处理多个目标
3. **查表法**：预计算弹道表，减少在线计算

---

## 4. 算法库介绍与接口说明

### 4.1 核心类设计

#### 4.1.1 Detector 类

```cpp
class Detector {
public:
    // 构造函数
    Detector(const std::string& config_path, bool debug = true);
    
    // 检测装甲板
    std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count = -1);
    
    // 检测单个装甲板（在 ROI 内）
    bool detect(Armor& armor, const cv::Mat& bgr_img);
};
```

**配置参数** (`configs/*.yaml`)：

```yaml
threshold: 200                    # 二值化阈值
max_angle_error: 30               # 灯条最大角度误差（度）
min_lightbar_ratio: 1.5           # 灯条最小长宽比
max_lightbar_ratio: 10.0          # 灯条最大长宽比
min_lightbar_length: 5.0         # 灯条最小长度（像素）
min_armor_ratio: 1.0              # 装甲板最小中线比
max_armor_ratio: 5.0             # 装甲板最大中线比
max_side_ratio: 2.0               # 灯条最大边长比
min_confidence: 0.8              # 分类器最小置信度
```

#### 4.1.2 Solver 类

```cpp
class Solver {
public:
    explicit Solver(const std::string& config_path);
    
    // PnP 解算装甲板姿态
    void solve(Armor& armor) const;
    
    // 设置云台姿态
    void set_R_gimbal2world(const Eigen::Quaterniond& q);
    
    // 重投影装甲板
    std::vector<cv::Point2f> reproject_armor(...);
};
```

**配置参数**：

```yaml
camera_matrix: [...]              # 相机内参
distort_coeffs: [...]            # 畸变系数
R_camera2gimbal: [...]           # 相机到云台旋转矩阵
t_camera2gimbal: [...]           # 相机到云台平移向量
R_gimbal2imubody: [...]          # 云台到 IMU 旋转矩阵
```

#### 4.1.3 Tracker 类

```cpp
class Tracker {
public:
    Tracker(const std::string& config_path, Solver& solver);
    
    // 跟踪目标
    std::list<Target> track(
        std::list<Armor>& armors, 
        std::chrono::steady_clock::time_point t,
        bool use_enemy_color = true);
    
    // 获取当前状态
    std::string state() const;
};
```

#### 4.1.4 ExtendedKalmanFilter 类

```cpp
class ExtendedKalmanFilter {
public:
    Eigen::VectorXd x;            // 状态向量
    Eigen::MatrixXd P;            // 协方差矩阵
    
    // 预测步骤
    Eigen::VectorXd predict(
        const Eigen::MatrixXd& F, 
        const Eigen::MatrixXd& Q);
    
    // 更新步骤
    Eigen::VectorXd update(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h = nullptr);
};
```

#### 4.1.5 PID 类

```cpp
class PID {
public:
    PID(float dt, float kp, float ki, float kd, 
        float max_out, float max_iout, bool angular = false);
    
    // 计算 PID 输出
    float calc(float set, float fdb);
};
```

#### 4.1.6 BallisticSolver 类

```cpp
class BallisticSolver {
public:
    explicit BallisticSolver(
        double shootSpeed = 12.0,
        double dragCoeff = 0.047,
        double gravity = 9.8);
    
    // 解算云台角度
    GimbalPose solveGimbalPose(cv::Point3d shootTarget);
};
```

#### 4.1.7 Buff 预测器类

```cpp
class Small_Predictor : public Predictor {
    void update(double angle, double nowtime) override;
    double predict(double delta_time) override;
};

class Big_Predictor : public Predictor {
    void update(double angle, double nowtime) override;
    double predict(double delta_time) override;
};
```

### 4.2 数据结构

#### 4.2.1 Armor 装甲板结构

```cpp
struct Armor {
    Color color;                  // 颜色（红/蓝）
    Lightbar left, right;         // 左右灯条
    cv::Point2f center;           // 中心点
    
    ArmorType type;               // 装甲类型（大/小）
    ArmorName name;              // 装甲编号
    double confidence;           // 置信度
    
    Eigen::Vector3d xyz_in_gimbal;    // 云台坐标系位置
    Eigen::Vector3d xyz_in_world;    // 世界坐标系位置
    Eigen::Vector3d ypr_in_gimbal;   // 云台坐标系姿态
    Eigen::Vector3d ypr_in_world;    // 世界坐标系姿态
    Eigen::Vector3d ypd_in_world;    // 球坐标（距离/方位角/俯仰角）
};
```

#### 4.2.2 Lightbar 灯条结构

```cpp
struct Lightbar {
    std::size_t id;
    Color color;
    cv::Point2f center, top, bottom;
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;
};
```

---

## 5. 算法结果展示

### 5.1 装甲板检测结果

**输入图像**：

```
┌────────────────────────────────────┐
│  ┌──────┐                          │
│  │ 灯条  │──────┐                   │
│  └──────┘      │                   │
│            ┌───┴────┐              │
│            │ 装甲板  │              │
│            └─────────┘              │
│     ┌──────┐                         │
│     │ 灯条  │                         │
│     └──────┘                         │
└────────────────────────────────────┘
```

**检测流程**：

```
原始图像 → 颜色分割 → 轮廓提取 → 灯条筛选 → 装甲板匹配 → 数字识别 → 输出
  BGR       二值化    几何特征   特征计算    NMS       分类器      Armor列表
```

**检测效果**：

```
┌────────────────────────────────────┐
│ [frame: 1234]                      │
│ ┌────────────────────────────────┐ │
│ │ ●──●                          │ │
│ │ │  │  ●────●                  │ │
│ │ │  │  │    │                  │ │
│ │ ●──●  │    │   ●────●        │ │
│ │        │    │   │    │        │ │
│ │        ●────●   ●────●        │ │
│ └────────────────────────────────┘ │
│ 检测到 3 个装甲板                   │
│ 步兵: 0.95, 哨兵: 0.88, 前哨: 0.92 │
└────────────────────────────────────┘
```

### 5.2 目标跟踪状态机

```
┌─────────┐    发现目标    ┌─────────┐
│ SEARCH  │ ─────────────▶ │ TRACK   │
│ 搜索状态 │                │ 跟踪状态 │
└─────────┘                └─────────┘
     ▲                         │
     │ 丢失                     │ 连续丢失
     │ 10帧                     │ 超过阈值
     │                         ▼
     │                   ┌─────────┐
     └───────────────────│ LOST    │
                         │ 丢失状态 │
                         └─────────┘
```

### 5.3 位姿解算可视化

```
┌────────────────────────────────────┐
│ Gimbal xyz: x=3.5m, y=-0.2m, z=5.0m│
│ World xyz: x=1.2m, y=4.8m, z=5.0m   │
│ Yaw: 15.3°, Pitch: -8.7°, Dist: 6.2m│
│                                    │
│    装甲板角点重投影对比             │
│    ●────────●  ← 原始角点          │
│    │        │                       │
│    │   ⊕    │   ← 中心点            │
│    │        │                       │
│    ●────────●  ← 重投影角点         │
│                                    │
│ 重投影误差: 3.2 像素                │
└────────────────────────────────────┘
```

### 5.4 能量机关预测结果

**小能量机关**：

```
角度(°) ──────────────────▶ 时间(s)
  0  60  120  180  240  300  360
  │   │   │   │   │   │   │
  ├───┼───┼───┼───┼───┼───┤ 实际角度
  ├─────────●─────────●──── 预测角度(卡尔曼)
```

**大能量机关**：

```
┌────────────────────────────────────┐
│ 状态估计:                          │
│   角度 θ: 127.5°                   │
│   角速度 v: 75.2°/s                │
│   幅值 a: 0.912                    │
│   角频率 ω: 1.942 rad/s            │
│   相位 φ: 0.523 rad                │
│                                    │
│ 预测命中点:                        │
│   预测时间: 0.15s                  │
│   预测角度: 142.3°                 │
└────────────────────────────────────┘
```

### 5.5 弹道补偿效果

**未补偿**：

```
实际弹着点     瞄准点
    ●           │
    │           │
    │           │
    │           ▼
    │         装甲板
    │        
    │      ●───●
    │      │   │
    │      ●───●
    └───────────── 子弹轨迹(下坠)
```

**补偿后**：

```
瞄准点     实际弹着点
  │           ●
  │           │
  │           │
  ▼           │
 装甲板      │
    ●───●    │
    │   │    │
    ●───●    │
           ────── 子弹轨迹(修正后)
```

---

## 6. 总结与展望

### 6.1 总结

本系统实现了 RoboMaster 比赛所需的完整视觉功能：

1. **多模态检测**：支持传统方法和深度学习方法
2. **精确跟踪**：基于状态机的目标跟踪
3. **高精度解算**：PnP + 重投影优化
4. **弹道补偿**：考虑空气阻力的弹道模型
5. **能量机关**：基于 EKF 的预测算法

### 6.2 未来优化方向

1. **模型优化**：引入更强的目标检测模型
2. **多目标跟踪**：实现完整的 MOT 算法
3. **端云协同**：结合边缘计算与服务器推理
4. **自适应阈值**：基于图像内容自动调整参数

---

*文档版本: 1.0*  
*最后更新: 2026-03-14*
