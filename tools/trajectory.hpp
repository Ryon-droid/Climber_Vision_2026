#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

#include <cmath>
#include <opencv2/opencv.hpp>

namespace tools
{
// 弹道解算方法枚举
enum class TrajectoryMethod
{
  WITHOUT_AIR_RESISTANCE,  // 不考虑空气阻力
  HERO_WITH_AIR_RESISTANCE,     // 考虑空气阻力
};

// Hero机器人专用的弹道解算器相关结构体和类
struct GimbalPose {
    double pitch;
    double yaw;
    double roll;
};



class BallisticSolver {
public:
    explicit BallisticSolver(double shootSpeed = 12.0,
                           double dragCoeff = 0.47 * 1.169 * (2 * M_PI * 0.02125 * 0.02125) / 2 / 0.041,
                           double gravity = 9.8);
    
    ~BallisticSolver() = default;
    
    void setShootSpeed(double speed);
    void setDragCoefficient(double dragCoeff);
    void setGravity(double gravity);
    
    GimbalPose solveGimbalPose(cv::Point3d shootTarget);
    
    double getShootSpeed() const;
    double getDragCoefficient() const;
    double getGravity() const;

public:
    double calculateFlightTime(double distance, double pitchAngle);
private:
    double solvePitchAngle(double distance, double deltaZ);
    double solveYawAngle(double x, double y);
    
    double speed_;
    double dragCoeff_;
    double gravity_;
};

struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 构造函数：支持多种弹道解算方法
  // v0 - 子弹初速度大小，单位：m/s
  // d  - 目标水平距离，单位：m
  // h  - 目标竖直高度，单位：m
  // method - 弹道解算方法（默认不考虑空气阻力）
  Trajectory(const double v0, const double d, const double h, 
             TrajectoryMethod method = TrajectoryMethod::WITHOUT_AIR_RESISTANCE);

private:
  // 不考虑空气阻力的解算方法
  void calculateWithoutAirResistance(const double v0, const double d, const double h);
  
  // 考虑空气阻力的解算方法
  void calculateHeroWithAirResistance(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP