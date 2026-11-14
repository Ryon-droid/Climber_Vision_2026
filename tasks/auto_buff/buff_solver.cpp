#include "buff_solver.hpp"

#include "tools/logger.hpp"

namespace auto_buff
{
cv::Matx33f Solver::rotation_matrix(double angle) const
{
  return cv::Matx33f(
    1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle));
}

void Solver::compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points)
{
  const std::vector<cv::Point3f> & base_points = object_points[0];
  for (int i = 1; i < 5; ++i) {
    double angle = i * THETA;
    cv::Matx33f R = rotation_matrix(angle);
    std::vector<cv::Point3f> rotated_points;
    for (const auto & point : base_points) {
      cv::Vec3f vec(point.x, point.y, point.z);
      cv::Vec3f rotated_vec = R * vec;
      rotated_points.emplace_back(rotated_vec[0], rotated_vec[1], rotated_vec[2]);
    }
    object_points[i] = rotated_points;
  }
}

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  // compute_rotated_points(OBJECT_POINTS);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::solve(std::optional<PowerRune> & ps) const
{
  // 检查是否有有效的PowerRune对象
  if (!ps.has_value()) {
    tools::logger()->debug("[Solver] 传入的PowerRune对象为空");
    return;
  }
  
  PowerRune& p = ps.value();
  
  // object_points.emplace_back(cv::Point3f(0, 0, 0));
  std::vector<cv::Point2f> image_points = p.target().points;
  // image_points.emplace_back(p.target().center);
  image_points.emplace_back(p.r_center);
  
  tools::logger()->debug("[Solver] 图像点坐标:");
  for (size_t i = 0; i < image_points.size(); ++i) {
    tools::logger()->debug("  点{}: ({}, {})", i, image_points[i].x, image_points[i].y);
  }

  std::vector<cv::Point2f> image_points_fourth(image_points.begin(), image_points.begin() + 4);
  std::vector<cv::Point3f> OBJECT_POINTS_FOURTH(OBJECT_POINTS.begin(), OBJECT_POINTS.begin() + 4);
  
  tools::logger()->debug("[Solver] 世界坐标点:");
  for (size_t i = 0; i < OBJECT_POINTS_FOURTH.size(); ++i) {
    tools::logger()->debug("  点{}: ({}, {}, {})", i, 
                          OBJECT_POINTS_FOURTH[i].x, 
                          OBJECT_POINTS_FOURTH[i].y, 
                          OBJECT_POINTS_FOURTH[i].z);
  }

  bool success = cv::solvePnP(
    OBJECT_POINTS_FOURTH, image_points_fourth, camera_matrix_, distort_coeffs_, rvec_, tvec_, false,
    cv::SOLVEPNP_IPPE);
    
  if (!success) {
    tools::logger()->warn("[Solver] solvePnP计算失败");
    // 初始化为零值以避免未定义行为
    p.xyz_in_world = Eigen::Vector3d(0, 0, 0);
    p.ypd_in_world = Eigen::Vector3d(0, 0, 0);
    p.ypr_in_world = Eigen::Vector3d(0, 0, 0);
    p.blade_xyz_in_world = Eigen::Vector3d(0, 0, 0);
    p.blade_ypd_in_world = Eigen::Vector3d(0, 0, 0);
    return;
  }

  tools::logger()->debug("[Solver] solvePnP计算成功");

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);
  
  tools::logger()->debug("[Solver] t_buff2camera: ({}, {}, {})", 
                        t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  Eigen::Vector3d blade_xyz_in_buff{{0, 0, 700e-3}};

  // buff -> camera
  Eigen::Vector3d xyz_in_camera = t_buff2camera;
  Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;
  
  tools::logger()->debug("[Solver] xyz_in_camera: ({}, {}, {})", 
                        xyz_in_camera[0], xyz_in_camera[1], xyz_in_camera[2]);
  tools::logger()->debug("[Solver] blade_xyz_in_camera: ({}, {}, {})", 
                        blade_xyz_in_camera[0], blade_xyz_in_camera[1], blade_xyz_in_camera[2]);

  // camera -> gimbal
  Eigen::Matrix3d R_buff2gimbal = R_camera2gimbal_ * R_buff2camera;
  Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;
  
  tools::logger()->debug("[Solver] xyz_in_gimbal: ({}, {}, {})", 
                        xyz_in_gimbal[0], xyz_in_gimbal[1], xyz_in_gimbal[2]);
  tools::logger()->debug("[Solver] blade_xyz_in_gimbal: ({}, {}, {})", 
                        blade_xyz_in_gimbal[0], blade_xyz_in_gimbal[1], blade_xyz_in_gimbal[2]);

  /// gimbal -> world
  Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;
  
  tools::logger()->debug("[Solver] R_gimbal2world_矩阵:");
  for (int i = 0; i < 3; ++i) {
    tools::logger()->debug("  [{}, {}, {}]", 
                          R_gimbal2world_(i, 0), R_gimbal2world_(i, 1), R_gimbal2world_(i, 2));
  }

  p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

  p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);

  p.ypr_in_world = tools::eulers(R_buff2world, 2, 1, 0);
  
  tools::logger()->debug("[Solver] 最终计算结果:");
  tools::logger()->debug("  xyz_in_world: ({}, {}, {})", 
                        p.xyz_in_world[0], p.xyz_in_world[1], p.xyz_in_world[2]);
  tools::logger()->debug("  ypd_in_world: ({}, {}, {})", 
                        p.ypd_in_world[0], p.ypd_in_world[1], p.ypd_in_world[2]);
  tools::logger()->debug("  ypr_in_world: ({}, {}, {})", 
                        p.ypr_in_world[0], p.ypr_in_world[1], p.ypr_in_world[2]);
}

// 调试用
cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  // buff坐标系(单位:m)到像素坐标系
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  world_points.push_back(x);
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points.back();
}

// xyz_in_world2xyz_in_pix
std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, double yaw, double row) const
{
  auto R_buff2world = tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, row));
  // clang-format on

  // get R_buff2camera t_buff2camera
  const Eigen::Vector3d & t_buff2world = xyz_in_world;
  Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_buff2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(OBJECT_POINTS, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}
}  // namespace auto_buff