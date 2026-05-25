#include "trajectory.hpp"

#include <cmath>
#include <limits>

namespace tools
{
constexpr double g = 9.7833;  // 重力加速度 (m/s²)

// BallisticSolver类实现
BallisticSolver::BallisticSolver(double shootSpeed, double dragCoeff, double gravity)
    : speed_(shootSpeed), dragCoeff_(dragCoeff), gravity_(gravity) {}

void BallisticSolver::setShootSpeed(double speed) {
    speed_ = speed;
}

void BallisticSolver::setDragCoefficient(double dragCoeff) {
    dragCoeff_ = dragCoeff;
}

void BallisticSolver::setGravity(double gravity) {
    gravity_ = gravity;
}

GimbalPose BallisticSolver::solveGimbalPose(cv::Point3d shootTarget) {
    GimbalPose gimbalPose;
    
    // Ensure y is not zero to avoid division by zero
    double y = (shootTarget.y == 0) ? 1e-6 : shootTarget.y;
    
    // Calculate yaw angle (simpler since it's in 2D plane)
    gimbalPose.yaw = solveYawAngle(shootTarget.x, y);
    
    // Calculate horizontal distance to target
    double distance = std::sqrt(shootTarget.x * shootTarget.x + y * y);
    
    // Calculate pitch angle with air resistance compensation
    double theta = solvePitchAngle(distance, shootTarget.z);
    
    // Convert pitch to desired output format (100x degrees for some systems)
    gimbalPose.pitch = (theta / M_PI * 180) * 100;
    gimbalPose.roll = 0.0; // Roll is typically not needed for ballistic calculations
    
    return gimbalPose;
}

double BallisticSolver::solvePitchAngle(double distance, double deltaZ) {
    // Initial estimate using simple ballistic trajectory without air resistance
    double theta = std::atan2(deltaZ, distance);
    
    // Iterative solution using Newton's method to account for air resistance
    for (int i = 0; i < 100; i++) {
        // Calculate flight time considering air resistance
        double flyTime = calculateFlightTime(distance, theta);
        
        // Calculate the error in deltaZ prediction
        double delta = deltaZ - speed_ * std::sin(theta) * flyTime / std::cos(theta) + 
                      0.5 * gravity_ * flyTime * flyTime / (std::cos(theta) * std::cos(theta));
        
        // Check if error is within tolerance
        if (std::fabs(delta) < 1e-6) {
            break;
        }
        
        // Update theta using Newton's method
        theta -= delta / (-(speed_ * flyTime) / std::pow(std::cos(theta), 2) + 
                         gravity_ * flyTime * flyTime * std::sin(theta) / (speed_ * speed_ * std::pow(std::cos(theta), 3)));
    }
    
    return theta;
}

double BallisticSolver::solveYawAngle(double x, double y) {
    // Yaw angle calculation (simpler since air resistance in horizontal plane is negligible)
    return std::atan(-x / y);
}

double BallisticSolver::calculateFlightTime(double distance, double pitchAngle) const {
    // Calculate flight time considering air resistance
    // Uses exponential air resistance model
    return (std::exp(dragCoeff_ * distance) - 1) / (dragCoeff_ * speed_ * std::cos(pitchAngle));
}

double BallisticSolver::calculateHeight(double distance, double pitchAngle) const {
    const double flyTime = calculateFlightTime(distance, pitchAngle);
    const double cos_pitch = std::cos(pitchAngle);
    if (std::fabs(cos_pitch) < 1e-6) {
        return std::numeric_limits<double>::infinity();
    }

    return speed_ * std::sin(pitchAngle) * flyTime / cos_pitch -
           0.5 * gravity_ * flyTime * flyTime / (cos_pitch * cos_pitch);
}

double BallisticSolver::getShootSpeed() const {
    return speed_;
}

double BallisticSolver::getDragCoefficient() const {
    return dragCoeff_;
}

double BallisticSolver::getGravity() const {
    return gravity_;
}

// 构造函数：支持多种弹道解算方法
Trajectory::Trajectory(const double v0, const double d, const double h, TrajectoryMethod method)
{
  // 根据选择的方法调用不同的计算函数
  switch (method) {
    case TrajectoryMethod::WITHOUT_AIR_RESISTANCE:
      calculateWithoutAirResistance(v0, d, h);
      break;
    case TrajectoryMethod::HERO_WITH_AIR_RESISTANCE:
      calculateHeroWithAirResistance(v0, d, h);
      break;
    default:
      calculateWithoutAirResistance(v0, d, h);
  }
}

// 不考虑空气阻力的解算方法（原方法）
void Trajectory::calculateWithoutAirResistance(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

// hero专用：考虑空气阻力的解算方法（使用BallisticSolver类）
void Trajectory::calculateHeroWithAirResistance(const double v0, const double d, const double h)
{
  // 创建BallisticSolver实例，使用子弹初速度v0
  BallisticSolver solver(v0);
  
  // 构建目标点：x=0（我们只考虑平面情况），y=d（水平距离），z=h（高度差）
  cv::Point3d target(0.0, d, h);
  
  // 调用solveGimbalPose方法计算弹道
  GimbalPose pose = solver.solveGimbalPose(target);
  
  // 转换回弧度（BallisticSolver返回的是100x角度）
  pitch = (pose.pitch / 100.0) * M_PI / 180.0;
  
  // 计算飞行时间
  fly_time = solver.calculateFlightTime(d, pitch);
  
  unsolvable = false;
}

}  // namespace tools
