/**
 * @file calibration_backend_opencv.cpp
 * @brief Implementation of OpenCV-based hand-eye calibration backend
 */

#include "easy_handeye2/calibration_backend_opencv.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace easy_handeye2
{

// Static member initialization
const std::map<std::string, CalibrationAlgorithm> CalibrationBackendOpenCV::algorithm_map_ = {
  {"Tsai-Lenz", CalibrationAlgorithm::TSAI_LENZ},
  {"Park", CalibrationAlgorithm::PARK},
  {"Horaud", CalibrationAlgorithm::HORAUD},
  {"Andreff", CalibrationAlgorithm::ANDREFF},
  {"Daniilidis", CalibrationAlgorithm::DANIILIDIS},
  {"Constrained", CalibrationAlgorithm::CONSTRAINED}
};

const std::map<CalibrationAlgorithm, std::string> CalibrationBackendOpenCV::algorithm_names_ = {
  {CalibrationAlgorithm::TSAI_LENZ, "Tsai-Lenz"},
  {CalibrationAlgorithm::PARK, "Park"},
  {CalibrationAlgorithm::HORAUD, "Horaud"},
  {CalibrationAlgorithm::ANDREFF, "Andreff"},
  {CalibrationAlgorithm::DANIILIDIS, "Daniilidis"},
  {CalibrationAlgorithm::CONSTRAINED, "Constrained"}
};

CalibrationBackendOpenCV::CalibrationBackendOpenCV()
: current_algorithm_(CalibrationAlgorithm::TSAI_LENZ)
{
}

std::vector<std::string> CalibrationBackendOpenCV::getAvailableAlgorithms()
{
  std::vector<std::string> algorithms;
  for (const auto& pair : algorithm_map_) {
    algorithms.push_back(pair.first);
  }
  return algorithms;
}

bool CalibrationBackendOpenCV::setAlgorithm(const std::string& algorithm)
{
  auto it = algorithm_map_.find(algorithm);
  if (it != algorithm_map_.end()) {
    current_algorithm_ = it->second;
    return true;
  }
  return false;
}

std::string CalibrationBackendOpenCV::getCurrentAlgorithm() const
{
  auto it = algorithm_names_.find(current_algorithm_);
  if (it != algorithm_names_.end()) {
    return it->second;
  }
  return "Unknown";
}

cv::HandEyeCalibrationMethod CalibrationBackendOpenCV::getOpenCVMethod() const
{
  switch (current_algorithm_) {
    case CalibrationAlgorithm::TSAI_LENZ:
      return cv::CALIB_HAND_EYE_TSAI;
    case CalibrationAlgorithm::PARK:
      return cv::CALIB_HAND_EYE_PARK;
    case CalibrationAlgorithm::HORAUD:
      return cv::CALIB_HAND_EYE_HORAUD;
    case CalibrationAlgorithm::ANDREFF:
      return cv::CALIB_HAND_EYE_ANDREFF;
    case CalibrationAlgorithm::DANIILIDIS:
      return cv::CALIB_HAND_EYE_DANIILIDIS;
    default:
      return cv::CALIB_HAND_EYE_TSAI;
  }
}

void CalibrationBackendOpenCV::transformToOpenCV(
  const geometry_msgs::msg::Transform& transform,
  cv::Mat& rotation,
  cv::Mat& translation)
{
  // Convert quaternion to rotation matrix using Eigen
  Eigen::Quaterniond q(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z);
  
  Eigen::Matrix3d rot_eigen = q.toRotationMatrix();
  
  // Convert to OpenCV format
  rotation = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rotation.at<double>(i, j) = rot_eigen(i, j);
    }
  }
  
  translation = cv::Mat(3, 1, CV_64F);
  translation.at<double>(0, 0) = transform.translation.x;
  translation.at<double>(1, 0) = transform.translation.y;
  translation.at<double>(2, 0) = transform.translation.z;
}

geometry_msgs::msg::Transform CalibrationBackendOpenCV::opencvToTransform(
  const cv::Mat& rotation,
  const cv::Mat& translation)
{
  geometry_msgs::msg::Transform transform;
  
  // Convert rotation matrix to Eigen
  Eigen::Matrix3d rot_eigen;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rot_eigen(i, j) = rotation.at<double>(i, j);
    }
  }
  
  // Convert to quaternion
  Eigen::Quaterniond q(rot_eigen);
  q.normalize();
  
  transform.rotation.w = q.w();
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  
  transform.translation.x = translation.at<double>(0, 0);
  transform.translation.y = translation.at<double>(1, 0);
  transform.translation.z = translation.at<double>(2, 0);
  
  return transform;
}

std::shared_ptr<HandeyeCalibration> CalibrationBackendOpenCV::computeCalibration(
  const HandeyeCalibrationParameters& params,
  const std::vector<CalibrationSample>& samples)
{
  std::cout << "\n[OpenCV Hand-Eye Calibration] Starting..." << std::endl;
  std::cout << "  Input samples: " << samples.size() << std::endl;
  std::cout << "  Algorithm: " << getCurrentAlgorithm() << std::endl;
  
  if (samples.size() < MIN_SAMPLES) {
    std::cout << "  ERROR: Not enough samples (min: " << MIN_SAMPLES << ")" << std::endl;
    return nullptr;
  }
  
  // Prepare data for OpenCV
  std::vector<cv::Mat> R_gripper2base, t_gripper2base;
  std::vector<cv::Mat> R_target2cam, t_target2cam;
  
  std::cout << "\n  Preparing data for OpenCV..." << std::endl;
  
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto& sample = samples[i];
    cv::Mat R_robot, t_robot;
    cv::Mat R_optical, t_optical;
    
    transformToOpenCV(sample.robot_transform.transform, R_robot, t_robot);
    transformToOpenCV(sample.optical_transform.transform, R_optical, t_optical);
    
    R_gripper2base.push_back(R_robot);
    t_gripper2base.push_back(t_robot);
    R_target2cam.push_back(R_optical);
    t_target2cam.push_back(t_optical);
  }
  
  std::cout << "  Data preparation complete." << std::endl;
  
  // Perform calibration
  cv::Mat R_cam2gripper, t_cam2gripper;
  
  std::cout << "\n  Calling cv::calibrateHandEye()..." << std::endl;
  
  try {
    cv::calibrateHandEye(
      R_gripper2base, t_gripper2base,
      R_target2cam, t_target2cam,
      R_cam2gripper, t_cam2gripper,
      getOpenCVMethod());
  } catch (const cv::Exception& e) {
    std::cout << "  ERROR: OpenCV exception: " << e.what() << std::endl;
    return nullptr;
  }
  
  std::cout << "  OpenCV calibration complete!" << std::endl;
  
  // Convert result to transform
  auto result_transform = opencvToTransform(R_cam2gripper, t_cam2gripper);
  
  // Print result
  std::cout << "\n  ======== Result ========" << std::endl;
  std::cout << "  Translation: [" 
            << result_transform.translation.x << ", "
            << result_transform.translation.y << ", "
            << result_transform.translation.z << "]" << std::endl;
  std::cout << "  Rotation (quat): ["
            << result_transform.rotation.x << ", "
            << result_transform.rotation.y << ", "
            << result_transform.rotation.z << ", "
            << result_transform.rotation.w << "]" << std::endl;
  std::cout << "  =========================\n" << std::endl;
  
  // Create calibration result
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform = result_transform;
  
  // Set frame IDs based on calibration type
  if (params.eye_on_hand) {
    transform_stamped.header.frame_id = params.robot_effector_frame;
  } else {
    transform_stamped.header.frame_id = params.robot_base_frame;
  }
  transform_stamped.child_frame_id = params.tracking_base_frame;
  
  auto calibration = std::make_shared<HandeyeCalibration>(params, transform_stamped);
  
  return calibration;
}

// ============================================================================
// CalibrationConstraints implementation
// ============================================================================

CalibrationConstraints CalibrationConstraints::fromYaml(const YAML::Node& node)
{
  CalibrationConstraints constraints;
  
  if (node["enabled"]) {
    constraints.enabled = node["enabled"].as<bool>();
  }
  
  // Load rotation constraints
  if (node["roll"] && !node["roll"].IsNull()) {
    constraints.roll = node["roll"].as<double>();
  }
  if (node["pitch"] && !node["pitch"].IsNull()) {
    constraints.pitch = node["pitch"].as<double>();
  }
  if (node["yaw"] && !node["yaw"].IsNull()) {
    constraints.yaw = node["yaw"].as<double>();
  }
  
  // Load translation constraints
  if (node["x"] && !node["x"].IsNull()) {
    constraints.x = node["x"].as<double>();
  }
  if (node["y"] && !node["y"].IsNull()) {
    constraints.y = node["y"].as<double>();
  }
  if (node["z"] && !node["z"].IsNull()) {
    constraints.z = node["z"].as<double>();
  }
  
  return constraints;
}

YAML::Node CalibrationConstraints::toYaml() const
{
  YAML::Node node;
  node["enabled"] = enabled;
  
  if (roll.has_value()) {
    node["roll"] = roll.value();
  } else {
    node["roll"] = YAML::Null;
  }
  
  if (pitch.has_value()) {
    node["pitch"] = pitch.value();
  } else {
    node["pitch"] = YAML::Null;
  }
  
  if (yaw.has_value()) {
    node["yaw"] = yaw.value();
  } else {
    node["yaw"] = YAML::Null;
  }
  
  if (x.has_value()) {
    node["x"] = x.value();
  } else {
    node["x"] = YAML::Null;
  }
  
  if (y.has_value()) {
    node["y"] = y.value();
  } else {
    node["y"] = YAML::Null;
  }
  
  if (z.has_value()) {
    node["z"] = z.value();
  } else {
    node["z"] = YAML::Null;
  }
  
  return node;
}

int CalibrationConstraints::getNumFreeParams() const
{
  int count = 0;
  if (!roll.has_value()) count++;
  if (!pitch.has_value()) count++;
  if (!yaw.has_value()) count++;
  if (!x.has_value()) count++;
  if (!y.has_value()) count++;
  if (!z.has_value()) count++;
  return count;
}

bool CalibrationConstraints::isValid() const
{
  // At least one parameter should be free
  return getNumFreeParams() > 0 && getNumFreeParams() <= 6;
}

GridSearchParams GridSearchParams::fromYaml(const YAML::Node& node)
{
  GridSearchParams p;
  if (!node) return p;
  if (node["trans_range"])       p.trans_range       = node["trans_range"].as<double>();
  if (node["trans_step"])        p.trans_step        = node["trans_step"].as<double>();
  if (node["rot_range"])         p.rot_range         = node["rot_range"].as<double>();
  if (node["rot_step"])          p.rot_step          = node["rot_step"].as<double>();
  if (node["gd_max_iterations"]) p.gd_max_iterations = node["gd_max_iterations"].as<int>();
  if (node["gd_learning_rate"])  p.gd_learning_rate  = node["gd_learning_rate"].as<double>();
  if (node["gd_delta"])          p.gd_delta          = node["gd_delta"].as<double>();
  return p;
}

// ============================================================================
// Constrained calibration implementation
// ============================================================================

namespace {

Eigen::Matrix3d eulerToRotation(double roll, double pitch, double yaw)
{
  // ZYX Euler angles (yaw, pitch, roll)
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q.toRotationMatrix();
}

double computeConstrainedResidual(
  const std::vector<CalibrationSample>& samples,
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  // Build camera-to-gripper transformation
  Eigen::Matrix3d R_cam2gripper = eulerToRotation(roll, pitch, yaw);
  Eigen::Vector3d t_cam2gripper(x, y, z);
  
  // Compute AX = XB residual for all sample pairs
  double total_residual = 0.0;
  int pair_count = 0;
  
  for (size_t i = 0; i < samples.size(); ++i) {
    for (size_t j = i + 1; j < samples.size(); ++j) {
      // Get robot transforms (gripper to base)
      const auto& robot_i = samples[i].robot_transform.transform;
      const auto& robot_j = samples[j].robot_transform.transform;
      
      // Get optical transforms (target to camera)
      const auto& optical_i = samples[i].optical_transform.transform;
      const auto& optical_j = samples[j].optical_transform.transform;
      
      // Convert to Eigen
      Eigen::Quaterniond q_robot_i(robot_i.rotation.w, robot_i.rotation.x, 
                                    robot_i.rotation.y, robot_i.rotation.z);
      Eigen::Quaterniond q_robot_j(robot_j.rotation.w, robot_j.rotation.x,
                                    robot_j.rotation.y, robot_j.rotation.z);
      Eigen::Vector3d t_robot_i(robot_i.translation.x, robot_i.translation.y, robot_i.translation.z);
      Eigen::Vector3d t_robot_j(robot_j.translation.x, robot_j.translation.y, robot_j.translation.z);
      
      Eigen::Quaterniond q_optical_i(optical_i.rotation.w, optical_i.rotation.x,
                                      optical_i.rotation.y, optical_i.rotation.z);
      Eigen::Quaterniond q_optical_j(optical_j.rotation.w, optical_j.rotation.x,
                                      optical_j.rotation.y, optical_j.rotation.z);
      Eigen::Vector3d t_optical_i(optical_i.translation.x, optical_i.translation.y, optical_i.translation.z);
      Eigen::Vector3d t_optical_j(optical_j.translation.x, optical_j.translation.y, optical_j.translation.z);
      
      // Compute relative transforms
      // A = inv(robot_i) * robot_j (relative robot motion)
      Eigen::Matrix3d R_robot_i = q_robot_i.toRotationMatrix();
      Eigen::Matrix3d R_robot_j = q_robot_j.toRotationMatrix();
      Eigen::Matrix3d R_A = R_robot_i.transpose() * R_robot_j;
      Eigen::Vector3d t_A = R_robot_i.transpose() * (t_robot_j - t_robot_i);
      
      // B = optical_i * inv(optical_j) (relative marker motion in camera)
      Eigen::Matrix3d R_optical_i = q_optical_i.toRotationMatrix();
      Eigen::Matrix3d R_optical_j = q_optical_j.toRotationMatrix();
      Eigen::Matrix3d R_B = R_optical_i * R_optical_j.transpose();
      Eigen::Vector3d t_B = t_optical_i - R_B * t_optical_j;
      
      // AX = XB constraint
      // R_A * R_X = R_X * R_B (rotation)
      // R_A * t_X + t_A = R_X * t_B + t_X (translation)
      
      // Rotation residual
      Eigen::Matrix3d R_AX = R_A * R_cam2gripper;
      Eigen::Matrix3d R_XB = R_cam2gripper * R_B;
      Eigen::Matrix3d R_diff = R_AX - R_XB;
      double rot_residual = R_diff.norm();
      
      // Translation residual  
      Eigen::Vector3d t_AX = R_A * t_cam2gripper + t_A;
      Eigen::Vector3d t_XB = R_cam2gripper * t_B + t_cam2gripper;
      Eigen::Vector3d t_diff = t_AX - t_XB;
      double trans_residual = t_diff.norm();
      
      total_residual += rot_residual + trans_residual;
      pair_count++;
    }
  }
  
  return pair_count > 0 ? total_residual / pair_count : 1e10;
}

}  // anonymous namespace

// 多线程网格搜索的任务结构
struct GridSearchTask {
  double x, y, z, roll, pitch, yaw;
};

struct GridSearchResult {
  double residual = 1e10;
  double x = 0, y = 0, z = 0;
  double roll = 0, pitch = 0, yaw = 0;
};

// 多线程工作函数
static void gridSearchWorker(
  const std::vector<CalibrationSample>& samples,
  const std::vector<GridSearchTask>& tasks,
  size_t start_idx,
  size_t end_idx,
  GridSearchResult& local_best,
  std::atomic<size_t>& progress_counter)
{
  for (size_t i = start_idx; i < end_idx; ++i) {
    const auto& task = tasks[i];
    double residual = computeConstrainedResidual(
      samples, task.x, task.y, task.z, task.roll, task.pitch, task.yaw);
    
    if (residual < local_best.residual) {
      local_best.residual = residual;
      local_best.x = task.x;
      local_best.y = task.y;
      local_best.z = task.z;
      local_best.roll = task.roll;
      local_best.pitch = task.pitch;
      local_best.yaw = task.yaw;
    }
    
    progress_counter.fetch_add(1, std::memory_order_relaxed);
  }
}

std::shared_ptr<HandeyeCalibration> CalibrationBackendOpenCV::computeConstrainedCalibration(
  const HandeyeCalibrationParameters& params,
  const std::vector<CalibrationSample>& samples,
  const CalibrationConstraints& constraints,
  const GridSearchParams& grid_params)
{
  if (samples.size() < MIN_SAMPLES) {
    return nullptr;
  }
  
  if (!constraints.enabled || !constraints.isValid()) {
    // Fall back to standard calibration
    return computeCalibration(params, samples);
  }
  
  // Initialize parameters with constraints or default values
  double roll = constraints.roll.value_or(0.0);
  double pitch = constraints.pitch.value_or(0.0);
  double yaw = constraints.yaw.value_or(0.0);
  double x = constraints.x.value_or(0.0);
  double y = constraints.y.value_or(0.0);
  double z = constraints.z.value_or(0.1);  // Default 10cm
  
  // Grid search + gradient descent for free parameters (from YAML grid_search config)
  const double trans_range = grid_params.trans_range;
  const double trans_step  = grid_params.trans_step;
  const double rot_range   = grid_params.rot_range;
  const double rot_step    = grid_params.rot_step;
  
  double best_residual = 1e10;
  double best_x = x, best_y = y, best_z = z;
  double best_roll = roll, best_pitch = pitch, best_yaw = yaw;
  
  // Grid search for free parameters
  auto search_x = [&]() -> std::vector<double> {
    if (constraints.x.has_value()) return {constraints.x.value()};
    std::vector<double> vals;
    for (double v = -trans_range; v <= trans_range; v += trans_step) vals.push_back(v);
    return vals;
  };
  
  auto search_y = [&]() -> std::vector<double> {
    if (constraints.y.has_value()) return {constraints.y.value()};
    std::vector<double> vals;
    for (double v = -trans_range; v <= trans_range; v += trans_step) vals.push_back(v);
    return vals;
  };
  
  auto search_z = [&]() -> std::vector<double> {
    if (constraints.z.has_value()) return {constraints.z.value()};
    std::vector<double> vals;
    for (double v = -trans_range; v <= trans_range; v += trans_step) vals.push_back(v);
    return vals;
  };
  
  auto search_roll = [&]() -> std::vector<double> {
    if (constraints.roll.has_value()) return {constraints.roll.value()};
    std::vector<double> vals;
    for (double v = -rot_range; v <= rot_range; v += rot_step) vals.push_back(v);
    return vals;
  };
  
  auto search_pitch = [&]() -> std::vector<double> {
    if (constraints.pitch.has_value()) return {constraints.pitch.value()};
    std::vector<double> vals;
    for (double v = -rot_range; v <= rot_range; v += rot_step) vals.push_back(v);
    return vals;
  };
  
  auto search_yaw = [&]() -> std::vector<double> {
    if (constraints.yaw.has_value()) return {constraints.yaw.value()};
    std::vector<double> vals;
    for (double v = -rot_range; v <= rot_range; v += rot_step) vals.push_back(v);
    return vals;
  };
  
  // Coarse grid search
  auto x_vals = search_x();
  auto y_vals = search_y();
  auto z_vals = search_z();
  auto roll_vals = search_roll();
  auto pitch_vals = search_pitch();
  auto yaw_vals = search_yaw();
  
  // 构建所有搜索任务
  std::vector<GridSearchTask> tasks;
  tasks.reserve(x_vals.size() * y_vals.size() * z_vals.size() * 
                roll_vals.size() * pitch_vals.size() * yaw_vals.size());
  
  for (double sx : x_vals) {
    for (double sy : y_vals) {
      for (double sz : z_vals) {
        for (double sr : roll_vals) {
          for (double sp : pitch_vals) {
            for (double sy_angle : yaw_vals) {
              tasks.push_back({sx, sy, sz, sr, sp, sy_angle});
            }
          }
        }
      }
    }
  }
  
  size_t total_iterations = tasks.size();
  
  // 确定线程数量
  unsigned int num_threads = std::thread::hardware_concurrency();
  if (num_threads == 0) num_threads = 4;  // 默认4线程
  num_threads = std::min(num_threads, static_cast<unsigned int>(tasks.size()));
  
  std::cout << "\n[Constrained Calibration] Multi-threaded Grid Search" << std::endl;
  std::cout << "  Total combinations: " << total_iterations << std::endl;
  std::cout << "  Using " << num_threads << " threads" << std::endl;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // 多线程并行搜索
  std::atomic<size_t> progress_counter(0);
  std::vector<std::thread> threads;
  std::vector<GridSearchResult> thread_results(num_threads);
  
  size_t chunk_size = (tasks.size() + num_threads - 1) / num_threads;
  
  for (unsigned int t = 0; t < num_threads; ++t) {
    size_t start_idx = t * chunk_size;
    size_t end_idx = std::min(start_idx + chunk_size, tasks.size());
    
    if (start_idx >= tasks.size()) break;
    
    threads.emplace_back(gridSearchWorker,
      std::cref(samples),
      std::cref(tasks),
      start_idx,
      end_idx,
      std::ref(thread_results[t]),
      std::ref(progress_counter));
  }
  
  // 进度监控线程
  std::thread progress_thread([&]() {
    size_t last_progress = 0;
    while (progress_counter.load() < total_iterations) {
      size_t current = progress_counter.load();
      if (current - last_progress >= total_iterations / 20 || current == total_iterations) {
        double progress = 100.0 * current / total_iterations;
        std::cout << "\r[Grid Search] Progress: " << std::fixed << std::setprecision(1) 
                 << progress << "% (" << current << "/" << total_iterations << ")" << std::flush;
        last_progress = current;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  
  // 等待所有工作线程完成
  for (auto& t : threads) {
    t.join();
  }
  progress_counter.store(total_iterations);  // 确保进度线程退出
  progress_thread.join();
  
  // 合并结果，找到全局最优
  for (const auto& result : thread_results) {
    if (result.residual < best_residual) {
      best_residual = result.residual;
      best_x = result.x;
      best_y = result.y;
      best_z = result.z;
      best_roll = result.roll;
      best_pitch = result.pitch;
      best_yaw = result.yaw;
    }
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  std::cout << "\n[Grid Search] Completed in " << duration.count() << " ms" << std::endl;
  std::cout << "[Grid Search] Best residual: " << best_residual << std::endl;
  
  // Fine tune with gradient descent (only for free parameters)
  const int    max_iterations = grid_params.gd_max_iterations;
  const double learning_rate  = grid_params.gd_learning_rate;
  const double delta          = grid_params.gd_delta;
  
  std::cout << "\n[Gradient Descent] Starting multi-threaded refinement..." << std::endl;
  auto gd_start_time = std::chrono::high_resolution_clock::now();
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    double current_residual = computeConstrainedResidual(
      samples, best_x, best_y, best_z, best_roll, best_pitch, best_yaw);
    
    // 并行计算各个自由参数的梯度
    std::vector<std::thread> grad_threads;
    double grad_x = 0, grad_y = 0, grad_z = 0, grad_roll = 0, grad_pitch = 0, grad_yaw = 0;
    
    // X gradient
    if (!constraints.x.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_x = (computeConstrainedResidual(samples, 
          best_x + delta, best_y, best_z, best_roll, best_pitch, best_yaw) - current_residual) / delta;
      });
    }
    
    // Y gradient
    if (!constraints.y.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_y = (computeConstrainedResidual(samples,
          best_x, best_y + delta, best_z, best_roll, best_pitch, best_yaw) - current_residual) / delta;
      });
    }
    
    // Z gradient
    if (!constraints.z.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_z = (computeConstrainedResidual(samples,
          best_x, best_y, best_z + delta, best_roll, best_pitch, best_yaw) - current_residual) / delta;
      });
    }
    
    // Roll gradient
    if (!constraints.roll.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_roll = (computeConstrainedResidual(samples,
          best_x, best_y, best_z, best_roll + delta, best_pitch, best_yaw) - current_residual) / delta;
      });
    }
    
    // Pitch gradient
    if (!constraints.pitch.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_pitch = (computeConstrainedResidual(samples,
          best_x, best_y, best_z, best_roll, best_pitch + delta, best_yaw) - current_residual) / delta;
      });
    }
    
    // Yaw gradient
    if (!constraints.yaw.has_value()) {
      grad_threads.emplace_back([&]() {
        grad_yaw = (computeConstrainedResidual(samples,
          best_x, best_y, best_z, best_roll, best_pitch, best_yaw + delta) - current_residual) / delta;
      });
    }
    
    // 等待所有梯度计算完成
    for (auto& t : grad_threads) {
      t.join();
    }
    
    // 更新参数
    if (!constraints.x.has_value()) best_x -= learning_rate * grad_x;
    if (!constraints.y.has_value()) best_y -= learning_rate * grad_y;
    if (!constraints.z.has_value()) best_z -= learning_rate * grad_z;
    if (!constraints.roll.has_value()) best_roll -= learning_rate * grad_roll;
    if (!constraints.pitch.has_value()) best_pitch -= learning_rate * grad_pitch;
    if (!constraints.yaw.has_value()) best_yaw -= learning_rate * grad_yaw;
    
    // 每10次迭代输出一次进度
    if (iter % 10 == 0 || iter == max_iterations - 1) {
      std::cout << "[Gradient Descent] Iteration " << iter 
               << ", residual: " << current_residual << std::endl;
    }
  }
  
  auto gd_end_time = std::chrono::high_resolution_clock::now();
  auto gd_duration = std::chrono::duration_cast<std::chrono::milliseconds>(gd_end_time - gd_start_time);
  std::cout << "[Gradient Descent] Completed in " << gd_duration.count() << " ms" << std::endl;
  
  // Build result transformation
  Eigen::Matrix3d R_result = eulerToRotation(best_roll, best_pitch, best_yaw);
  Eigen::Quaterniond q_result(R_result);
  q_result.normalize();
  
  // 输出最终标定结果
  std::cout << "\n========== Constrained Calibration Result ==========" << std::endl;
  std::cout << "Translation:" << std::endl;
  std::cout << "  x: " << best_x << " m" << std::endl;
  std::cout << "  y: " << best_y << " m" << std::endl;
  std::cout << "  z: " << best_z << " m" << std::endl;
  std::cout << "Rotation (Euler angles):" << std::endl;
  std::cout << "  roll:  " << best_roll << " rad (" << (best_roll * 180.0 / M_PI) << " deg)" << std::endl;
  std::cout << "  pitch: " << best_pitch << " rad (" << (best_pitch * 180.0 / M_PI) << " deg)" << std::endl;
  std::cout << "  yaw:   " << best_yaw << " rad (" << (best_yaw * 180.0 / M_PI) << " deg)" << std::endl;
  std::cout << "Quaternion:" << std::endl;
  std::cout << "  [x, y, z, w] = [" << q_result.x() << ", " << q_result.y() 
           << ", " << q_result.z() << ", " << q_result.w() << "]" << std::endl;
  std::cout << "Final residual: " << best_residual << std::endl;
  std::cout << "====================================================\n" << std::endl;
  
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = best_x;
  transform_stamped.transform.translation.y = best_y;
  transform_stamped.transform.translation.z = best_z;
  transform_stamped.transform.rotation.w = q_result.w();
  transform_stamped.transform.rotation.x = q_result.x();
  transform_stamped.transform.rotation.y = q_result.y();
  transform_stamped.transform.rotation.z = q_result.z();
  
  // Set frame IDs based on calibration type
  if (params.eye_on_hand) {
    transform_stamped.header.frame_id = params.robot_effector_frame;
  } else {
    transform_stamped.header.frame_id = params.robot_base_frame;
  }
  transform_stamped.child_frame_id = params.tracking_base_frame;
  
  auto calibration = std::make_shared<HandeyeCalibration>(params, transform_stamped);
  
  return calibration;
}

}  // namespace easy_handeye2
