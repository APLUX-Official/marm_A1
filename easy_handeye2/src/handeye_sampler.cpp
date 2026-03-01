/**
 * @file handeye_sampler.cpp
 * @brief Implementation of TF sampler for hand-eye calibration
 */

#include "easy_handeye2/handeye_sampler.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>

namespace easy_handeye2
{

HandeyeSampler::HandeyeSampler(
  rclcpp::Node* node,
  const HandeyeCalibrationParameters& params)
: node_(node), params_(params)
{
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool HandeyeSampler::waitForFrames(const std::chrono::seconds& timeout)
{
  auto start_time = node_->now();
  rclcpp::Rate rate(10.0);
  
  while (rclcpp::ok()) {
    try {
      // Check robot frames
      tf_buffer_->lookupTransform(
        params_.robot_base_frame,
        params_.robot_effector_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      
      // Check tracking frames
      tf_buffer_->lookupTransform(
        params_.tracking_base_frame,
        params_.tracking_marker_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      
      RCLCPP_INFO(node_->get_logger(), "All required TF frames are available");
      return true;
      
    } catch (const tf2::TransformException& ex) {
      auto elapsed = (node_->now() - start_time).seconds();
      if (elapsed > static_cast<double>(timeout.count())) {
        RCLCPP_ERROR(node_->get_logger(), 
          "Timeout waiting for TF frames: %s", ex.what());
        return false;
      }
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Waiting for TF frames: %s", ex.what());
    }
    
    rate.sleep();
  }
  
  return false;
}

bool HandeyeSampler::getCurrentTransforms(CalibrationSample& sample)
{
  try {
    auto now = node_->now();
    
    // Get robot transform
    // For eye-on-hand: base -> effector
    // For eye-on-base: effector -> base (inverted)
    if (params_.eye_on_hand) {
      sample.robot_transform = tf_buffer_->lookupTransform(
        params_.robot_base_frame,
        params_.robot_effector_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(5.0));
    } else {
      // For eye-on-base, we need effector -> base
      sample.robot_transform = tf_buffer_->lookupTransform(
        params_.robot_effector_frame,
        params_.robot_base_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(5.0));
    }
    
    // Get optical transform: camera -> marker
    sample.optical_transform = tf_buffer_->lookupTransform(
      params_.tracking_base_frame,
      params_.tracking_marker_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(5.0));
    
    sample.timestamp = now;
    
    return true;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Failed to get transforms: %s", ex.what());
    return false;
  }
}

bool HandeyeSampler::takeSample()
{
  RCLCPP_INFO(node_->get_logger(), "Taking a sample...");
  
  CalibrationSample sample;
  if (!getCurrentTransforms(sample)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to take sample");
    return false;
  }
  
  samples_.push_back(sample);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Sample %zu taken successfully", samples_.size());
  
  // Log sample details
  const auto& robot_t = sample.robot_transform.transform.translation;
  const auto& optical_t = sample.optical_transform.transform.translation;
  
  RCLCPP_DEBUG(node_->get_logger(),
    "Robot: [%.3f, %.3f, %.3f], Optical: [%.3f, %.3f, %.3f]",
    robot_t.x, robot_t.y, robot_t.z,
    optical_t.x, optical_t.y, optical_t.z);
  
  return true;
}

bool HandeyeSampler::removeSample(size_t index)
{
  if (index >= samples_.size()) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Invalid sample index: %zu (have %zu samples)", index, samples_.size());
    return false;
  }
  
  samples_.erase(samples_.begin() + static_cast<long>(index));
  RCLCPP_INFO(node_->get_logger(), 
    "Sample %zu removed. %zu samples remaining", index, samples_.size());
  return true;
}

bool HandeyeSampler::removeLastSample()
{
  if (samples_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No samples to remove");
    return false;
  }
  
  return removeSample(samples_.size() - 1);
}

void HandeyeSampler::clearSamples()
{
  samples_.clear();
  RCLCPP_INFO(node_->get_logger(), "All samples cleared");
}

std::string HandeyeSampler::getDefaultSamplesFilepath(const std::string& name)
{
  const char* home = std::getenv("HOME");
  std::string base_dir = std::string(home) + "/.ros/easy_handeye2";
  
  // Create directory if it doesn't exist
  mkdir(base_dir.c_str(), 0755);
  
  return base_dir + "/samples_" + name + ".yaml";
}

bool HandeyeSampler::saveSamplesToFile(const std::string& filepath) const
{
  std::string actual_path = filepath;
  if (actual_path.empty()) {
    actual_path = getDefaultSamplesFilepath(params_.name);
  }
  
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    // Save calibration parameters info
    out << YAML::Key << "calibration_info" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << params_.name;
    out << YAML::Key << "eye_on_hand" << YAML::Value << params_.eye_on_hand;
    out << YAML::Key << "robot_base_frame" << YAML::Value << params_.robot_base_frame;
    out << YAML::Key << "robot_effector_frame" << YAML::Value << params_.robot_effector_frame;
    out << YAML::Key << "tracking_base_frame" << YAML::Value << params_.tracking_base_frame;
    out << YAML::Key << "tracking_marker_frame" << YAML::Value << params_.tracking_marker_frame;
    out << YAML::EndMap;
    
    // Save samples
    out << YAML::Key << "samples_count" << YAML::Value << samples_.size();
    out << YAML::Key << "samples" << YAML::Value << YAML::BeginSeq;
    
    for (size_t i = 0; i < samples_.size(); ++i) {
      const auto& sample = samples_[i];
      out << YAML::BeginMap;
      out << YAML::Key << "index" << YAML::Value << i;
      
      // Robot transform
      out << YAML::Key << "robot_transform" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "frame_id" << YAML::Value << sample.robot_transform.header.frame_id;
      out << YAML::Key << "child_frame_id" << YAML::Value << sample.robot_transform.child_frame_id;
      out << YAML::Key << "translation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << sample.robot_transform.transform.translation.x;
      out << YAML::Key << "y" << YAML::Value << sample.robot_transform.transform.translation.y;
      out << YAML::Key << "z" << YAML::Value << sample.robot_transform.transform.translation.z;
      out << YAML::EndMap;
      out << YAML::Key << "rotation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << sample.robot_transform.transform.rotation.x;
      out << YAML::Key << "y" << YAML::Value << sample.robot_transform.transform.rotation.y;
      out << YAML::Key << "z" << YAML::Value << sample.robot_transform.transform.rotation.z;
      out << YAML::Key << "w" << YAML::Value << sample.robot_transform.transform.rotation.w;
      out << YAML::EndMap;
      out << YAML::EndMap;
      
      // Optical transform
      out << YAML::Key << "optical_transform" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "frame_id" << YAML::Value << sample.optical_transform.header.frame_id;
      out << YAML::Key << "child_frame_id" << YAML::Value << sample.optical_transform.child_frame_id;
      out << YAML::Key << "translation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << sample.optical_transform.transform.translation.x;
      out << YAML::Key << "y" << YAML::Value << sample.optical_transform.transform.translation.y;
      out << YAML::Key << "z" << YAML::Value << sample.optical_transform.transform.translation.z;
      out << YAML::EndMap;
      out << YAML::Key << "rotation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << sample.optical_transform.transform.rotation.x;
      out << YAML::Key << "y" << YAML::Value << sample.optical_transform.transform.rotation.y;
      out << YAML::Key << "z" << YAML::Value << sample.optical_transform.transform.rotation.z;
      out << YAML::Key << "w" << YAML::Value << sample.optical_transform.transform.rotation.w;
      out << YAML::EndMap;
      out << YAML::EndMap;
      
      out << YAML::EndMap;  // End sample
    }
    
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    std::ofstream fout(actual_path);
    if (!fout.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open file for writing: %s", actual_path.c_str());
      return false;
    }
    fout << out.c_str();
    fout.close();
    
    RCLCPP_INFO(node_->get_logger(), "Samples saved to: %s (%zu samples)", actual_path.c_str(), samples_.size());
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save samples: %s", e.what());
    return false;
  }
}

bool HandeyeSampler::loadSamplesFromFile(const std::string& filepath)
{
  std::string actual_path = filepath;
  if (actual_path.empty()) {
    actual_path = getDefaultSamplesFilepath(params_.name);
  }
  
  try {
    std::ifstream fin(actual_path);
    if (!fin.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Sample file not found: %s", actual_path.c_str());
      return false;
    }
    
    YAML::Node config = YAML::LoadFile(actual_path);
    
    // Clear existing samples
    samples_.clear();
    
    if (!config["samples"]) {
      RCLCPP_WARN(node_->get_logger(), "No samples found in file: %s", actual_path.c_str());
      return false;
    }
    
    for (const auto& sample_node : config["samples"]) {
      CalibrationSample sample;
      
      // Load robot transform
      auto robot_tf = sample_node["robot_transform"];
      sample.robot_transform.header.frame_id = robot_tf["frame_id"].as<std::string>("");
      sample.robot_transform.child_frame_id = robot_tf["child_frame_id"].as<std::string>("");
      sample.robot_transform.transform.translation.x = robot_tf["translation"]["x"].as<double>();
      sample.robot_transform.transform.translation.y = robot_tf["translation"]["y"].as<double>();
      sample.robot_transform.transform.translation.z = robot_tf["translation"]["z"].as<double>();
      sample.robot_transform.transform.rotation.x = robot_tf["rotation"]["x"].as<double>();
      sample.robot_transform.transform.rotation.y = robot_tf["rotation"]["y"].as<double>();
      sample.robot_transform.transform.rotation.z = robot_tf["rotation"]["z"].as<double>();
      sample.robot_transform.transform.rotation.w = robot_tf["rotation"]["w"].as<double>();
      
      // Load optical transform
      auto optical_tf = sample_node["optical_transform"];
      sample.optical_transform.header.frame_id = optical_tf["frame_id"].as<std::string>("");
      sample.optical_transform.child_frame_id = optical_tf["child_frame_id"].as<std::string>("");
      sample.optical_transform.transform.translation.x = optical_tf["translation"]["x"].as<double>();
      sample.optical_transform.transform.translation.y = optical_tf["translation"]["y"].as<double>();
      sample.optical_transform.transform.translation.z = optical_tf["translation"]["z"].as<double>();
      sample.optical_transform.transform.rotation.x = optical_tf["rotation"]["x"].as<double>();
      sample.optical_transform.transform.rotation.y = optical_tf["rotation"]["y"].as<double>();
      sample.optical_transform.transform.rotation.z = optical_tf["rotation"]["z"].as<double>();
      sample.optical_transform.transform.rotation.w = optical_tf["rotation"]["w"].as<double>();
      
      sample.timestamp = node_->now();
      samples_.push_back(sample);
    }
    
    RCLCPP_INFO(node_->get_logger(), 
      "Loaded %zu samples from: %s", samples_.size(), actual_path.c_str());
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load samples: %s", e.what());
    return false;
  }
}

}  // namespace easy_handeye2
