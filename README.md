# Climber_2026 自瞄工作空间

## 前言
分为四层：src、tasks、io、tools   
参考学习同济25开源   
主要作为上场保底代码和26赛季的学习平台  

## 1. 安装依赖
剔除了**cere库**依赖     
[OPENVIO](https://docs.openvino.ai/2025/get-started/install-openvino/install-openvino-archive-linux.html)   \
<!-- [Cere]<http://ceres-solver.org/installation.html>   \
**24.04中cere要把第三方库也clone全**
```bash
git clone --recurse-submodules https://ceres-solver.googlesource.com/ceres-solver
``` -->

其他：

```shell
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```


## 2. 编译
```bash
cmake -B build
make -C build/ -j`nproc`
```
## 3. 运行识别测试程序
```bash
./build/auto_aim_test configs/test.yaml
```

## 4. 相机调用
改回去了    
<!-- 修改了相机调用逻辑，通过相机名访问设备   
需在yaml文件里改变  
```yaml
user_id: your_camera_name
``` -->

## 5. tm的坐标系转换
1. OpenCV坐标系视角下：    
    前方：z
    右方：x
    下方：y

2. 手眼标定已经包含了云台坐标到相机坐标的转换：   
    云台的x轴对应相机的z轴
    云台的y轴对应相机的-x轴
    云台的z轴对应相机的-y轴

3. 需要获得imu到云台的坐标转换关系：  
    ```yaml
    R_gimbal2imubody : []
    ```
    旋转矩阵一般可靠但平移矩阵存在一定误差
4. 输出的yaw、pitch是直接将**相机坐标系**的y、x转换的，所以*正方向*以相机坐标系的y、x为参考


## 6. USB相机（感知）：
杰瑞微通 720p30hz       
```shell
#查看USB相机的参数范围：
v4l2-ctl -d /dev/video0 --list-ctrls
```

通过对锐度进行设置绑定相机编号：
```shell
# 查看当前值
v4l2-ctl -d /dev/video2 --get-ctrl=sharpness

# 设置为2(左侧摄像头)
v4l2-ctl -d /dev/video2 --set-ctrl=sharpness=2

# 设置为3(右侧摄像头)
v4l2-ctl -d /dev/video4 --set-ctrl=sharpness=3
```


## 参考
深度参考同济25开源，基于**工具层:tools**，修改**硬件层：io、功能层：tasks**,重写**应用层：src**   
[同济源码链接](https://github.com/TongjiSuperPower/sp_vision_25.git)

