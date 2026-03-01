/**
 * @file handeye_server_node.cpp
 * @brief Simplified hand-eye calibration server node
 * 
 * All parameters are loaded from YAML config file.
 * Only provides core calibration services:
 *   - ~/take_sample
 *   - ~/compute_calibration
 */
// Note: calibration is auto-saved to YAML after successful computation

#include "easy_handeye2/handeye_server.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace easy_handeye2
{

HandeyeServer::HandeyeServer(const rclcpp::NodeOptions& options)
: Node("handeye_calibration_server", options)
{
  // Only one ROS parameter: path to YAML config file
  this->declare_parameter<std::string>("config_file", "");
  
  std::string config_file = this->get_parameter("config_file").as_string();
  if (config_file.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No config_file parameter specified. Exiting.");
    return;
  }
  
  // Load all configuration from YAML
  loadConfigFromYaml(config_file);
  
  // Initialize calibration backend with algorithm from YAML
  calibration_backend_ = std::make_shared<CalibrationBackendOpenCV>();
  // algorithm name is stored in params_ during loadConfigFromYaml via a local variable
  // We need a member or local to pass it - let's re-read from YAML
  {
    YAML::Node cfg = YAML::LoadFile(config_file);
    std::string algorithm = "Tsai-Lenz";
    if (cfg["algorithm"] && cfg["algorithm"]["name"]) {
      algorithm = cfg["algorithm"]["name"].as<std::string>();
    }
    if (!calibration_backend_->setAlgorithm(algorithm)) {
      RCLCPP_WARN(this->get_logger(),
        "Unknown algorithm '%s', using default (Tsai-Lenz)", algorithm.c_str());
    }
  }
  
  // Initialize sampler
  sampler_ = std::make_shared<HandeyeSampler>(this, params_);
  
  // Try to load previously saved samples
  std::string samples_file = HandeyeSampler::getDefaultSamplesFilepath(params_.name);
  if (sampler_->loadSamplesFromFile(samples_file)) {
    RCLCPP_INFO(this->get_logger(),
      "Loaded %zu previously saved samples from: %s",
      sampler_->getSampleCount(), samples_file.c_str());
  }
  
  // Initialize services
  initializeServices();
  
  RCLCPP_INFO(this->get_logger(), "Hand-eye calibration server initialized");
  RCLCPP_INFO(this->get_logger(), "  Eye-on-hand: %s", params_.eye_on_hand ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Robot base frame: %s", params_.robot_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  Robot effector frame: %s", params_.robot_effector_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  Tracking base frame: %s", params_.tracking_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  Tracking marker frame: %s", params_.tracking_marker_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  Algorithm: %s", calibration_backend_->getCurrentAlgorithm().c_str());
  RCLCPP_INFO(this->get_logger(), "  Current samples: %zu", sampler_->getSampleCount());
}

void HandeyeServer::loadConfigFromYaml(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    
    // Load calibration parameters
    if (config["calibration"]) {
      auto calib = config["calibration"];
      if (calib["name"]) {
        params_.name = calib["name"].as<std::string>();
      }
      if (calib["eye_on_hand"]) {
        params_.eye_on_hand = calib["eye_on_hand"].as<bool>();
      }
      if (calib["robot_base_frame"]) {
        params_.robot_base_frame = calib["robot_base_frame"].as<std::string>();
      }
      if (calib["robot_effector_frame"]) {
        params_.robot_effector_frame = calib["robot_effector_frame"].as<std::string>();
      }
      if (calib["tracking_base_frame"]) {
        params_.tracking_base_frame = calib["tracking_base_frame"].as<std::string>();
      }
      if (calib["tracking_marker_frame"]) {
        params_.tracking_marker_frame = calib["tracking_marker_frame"].as<std::string>();
      }
    }
    
    // Load constraint settings
    if (config["constraints"]) {
      constraints_ = CalibrationConstraints::fromYaml(config["constraints"]);
      
      if (constraints_.enabled) {
        RCLCPP_INFO(this->get_logger(), "Constraints loaded:");
        RCLCPP_INFO(this->get_logger(), "  roll: %s",
          constraints_.roll.has_value() ? std::to_string(constraints_.roll.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  pitch: %s",
          constraints_.pitch.has_value() ? std::to_string(constraints_.pitch.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  yaw: %s",
          constraints_.yaw.has_value() ? std::to_string(constraints_.yaw.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  x: %s",
          constraints_.x.has_value() ? std::to_string(constraints_.x.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  y: %s",
          constraints_.y.has_value() ? std::to_string(constraints_.y.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  z: %s",
          constraints_.z.has_value() ? std::to_string(constraints_.z.value()).c_str() : "free");
        RCLCPP_INFO(this->get_logger(), "  Free parameters: %d", constraints_.getNumFreeParams());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded configuration from: %s", config_file.c_str());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to load config file '%s': %s", config_file.c_str(), e.what());
  }
}

void HandeyeServer::initializeServices()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  
  take_sample_srv_ = this->create_service<easy_handeye2::srv::TakeSample>(
    "~/take_sample",
    std::bind(&HandeyeServer::takeSampleCallback, this, _1, _2));
  
  compute_calibration_srv_ = this->create_service<easy_handeye2::srv::ComputeCalibration>(
    "~/compute_calibration",
    std::bind(&HandeyeServer::computeCalibrationCallback, this, _1, _2));
}

void HandeyeServer::takeSampleCallback(
  const std::shared_ptr<easy_handeye2::srv::TakeSample::Request> /*request*/,
  std::shared_ptr<easy_handeye2::srv::TakeSample::Response> response)
{
  response->success = sampler_->takeSample();
  
  RCLCPP_INFO(this->get_logger(),
    "Sample count: %zu", sampler_->getSampleCount());
  
  // Auto-save samples to YAML after each successful sample
  if (response->success) {
    if (sampler_->saveSamplesToFile()) {
      RCLCPP_INFO(this->get_logger(), "Samples auto-saved to YAML file");
    }
  }
}

void HandeyeServer::computeCalibrationCallback(
  const std::shared_ptr<easy_handeye2::srv::ComputeCalibration::Request> /*request*/,
  std::shared_ptr<easy_handeye2::srv::ComputeCalibration::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "            Starting Hand-Eye Calibration");
  RCLCPP_INFO(this->get_logger(), "============================================================");
  
  // Try to load samples from file if no samples in memory
  if (sampler_->getSampleCount() == 0) {
    RCLCPP_INFO(this->get_logger(), "[Step 0] No samples in memory, trying to load from YAML file...");
    if (sampler_->loadSamplesFromFile()) {
      RCLCPP_INFO(this->get_logger(), "[Step 0] Successfully loaded %zu samples from file",
        sampler_->getSampleCount());
    } else {
      RCLCPP_WARN(this->get_logger(), "[Step 0] No saved samples found");
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "[Step 1] Configuration:");
  RCLCPP_INFO(this->get_logger(), "  - Total samples: %zu", sampler_->getSampleCount());
  RCLCPP_INFO(this->get_logger(), "  - Algorithm: %s", calibration_backend_->getCurrentAlgorithm().c_str());
  RCLCPP_INFO(this->get_logger(), "  - Eye-on-hand: %s", params_.eye_on_hand ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  - Robot base frame: %s", params_.robot_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Robot effector frame: %s", params_.robot_effector_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Tracking base frame: %s", params_.tracking_base_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Tracking marker frame: %s", params_.tracking_marker_frame.c_str());
  
  // Print all samples
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "[Step 2] Sample Data:");
  const auto& samples = sampler_->getSamples();
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto& sample = samples[i];
    const auto& rt = sample.robot_transform.transform;
    const auto& ot = sample.optical_transform.transform;
    
    RCLCPP_INFO(this->get_logger(), "  Sample %zu:", i + 1);
    RCLCPP_INFO(this->get_logger(), "    Robot:   T=[%.4f, %.4f, %.4f] Q=[%.4f, %.4f, %.4f, %.4f]",
      rt.translation.x, rt.translation.y, rt.translation.z,
      rt.rotation.x, rt.rotation.y, rt.rotation.z, rt.rotation.w);
    RCLCPP_INFO(this->get_logger(), "    Optical: T=[%.4f, %.4f, %.4f] Q=[%.4f, %.4f, %.4f, %.4f]",
      ot.translation.x, ot.translation.y, ot.translation.z,
      ot.rotation.x, ot.rotation.y, ot.rotation.z, ot.rotation.w);
  }
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "[Step 3] Computing calibration...");
  
  // Use constrained calibration if algorithm is "Constrained" and constraints are enabled
  if (calibration_backend_->getCurrentAlgorithm() == "Constrained" && constraints_.enabled) {
    RCLCPP_INFO(this->get_logger(), "  Using constrained calibration with %d free parameters",
      constraints_.getNumFreeParams());
    RCLCPP_INFO(this->get_logger(), "  Constraints: roll=%s, pitch=%s, yaw=%s, x=%s, y=%s, z=%s",
      constraints_.roll.has_value() ? std::to_string(constraints_.roll.value()).c_str() : "free",
      constraints_.pitch.has_value() ? std::to_string(constraints_.pitch.value()).c_str() : "free",
      constraints_.yaw.has_value() ? std::to_string(constraints_.yaw.value()).c_str() : "free",
      constraints_.x.has_value() ? std::to_string(constraints_.x.value()).c_str() : "free",
      constraints_.y.has_value() ? std::to_string(constraints_.y.value()).c_str() : "free",
      constraints_.z.has_value() ? std::to_string(constraints_.z.value()).c_str() : "free");
    last_calibration_ = calibration_backend_->computeConstrainedCalibration(
      params_, sampler_->getSamples(), constraints_);
  } else {
    RCLCPP_INFO(this->get_logger(), "  Using standard OpenCV hand-eye calibration...");
    last_calibration_ = calibration_backend_->computeCalibration(
      params_, sampler_->getSamples());
  }
  
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "[Step 4] Calibration Result:");
  
  if (last_calibration_) {
    response->valid = true;
    response->message = "Calibration computed successfully";
    
    const auto& transform = last_calibration_->getTransform();
    const auto& t = transform.transform.translation;
    const auto& r = transform.transform.rotation;

    RCLCPP_INFO(this->get_logger(), "  ====== CALIBRATION SUCCESS ======");
    RCLCPP_INFO(this->get_logger(), "  Translation (m):");
    RCLCPP_INFO(this->get_logger(), "    x: %.6f", t.x);
    RCLCPP_INFO(this->get_logger(), "    y: %.6f", t.y);
    RCLCPP_INFO(this->get_logger(), "    z: %.6f", t.z);
    RCLCPP_INFO(this->get_logger(), "  Rotation (quaternion):");
    RCLCPP_INFO(this->get_logger(), "    x: %.6f", r.x);
    RCLCPP_INFO(this->get_logger(), "    y: %.6f", r.y);
    RCLCPP_INFO(this->get_logger(), "    z: %.6f", r.z);
    RCLCPP_INFO(this->get_logger(), "    w: %.6f", r.w);
    
    // Convert to Euler angles for readability
    double sinr_cosp = 2.0 * (r.w * r.x + r.y * r.z);
    double cosr_cosp = 1.0 - 2.0 * (r.x * r.x + r.y * r.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (r.w * r.y - r.z * r.x);
    double pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);
    
    double siny_cosp = 2.0 * (r.w * r.z + r.x * r.y);
    double cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    RCLCPP_INFO(this->get_logger(), "  Rotation (Euler angles, rad):");
    RCLCPP_INFO(this->get_logger(), "    roll:  %.6f (%.2f deg)", roll, roll * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "    pitch: %.6f (%.2f deg)", pitch, pitch * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "    yaw:   %.6f (%.2f deg)", yaw, yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  ===================================");

    // Auto-save calibration to YAML file
    std::string save_path = HandeyeCalibration::getDefaultFilepath(params_.name);
    if (last_calibration_->saveToFile(save_path)) {
      RCLCPP_INFO(this->get_logger(), "  Calibration auto-saved to: %s", save_path.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "  Failed to auto-save calibration to: %s", save_path.c_str());
    }
    
  } else {
    response->valid = false;
    response->message = "Not enough samples";
    RCLCPP_ERROR(this->get_logger(), "  ====== CALIBRATION FAILED ======");
    RCLCPP_ERROR(this->get_logger(),
      "  Need at least %zu samples (have %zu)",
      CalibrationBackendOpenCV::MIN_SAMPLES,
      sampler_->getSampleCount());
  }
  
  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "");
}

}  // namespace easy_handeye2

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<easy_handeye2::HandeyeServer>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
