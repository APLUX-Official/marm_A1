/**
 * @file handeye_calibration.cpp
 * @brief Implementation of hand-eye calibration data structures
 */

#include "easy_handeye2/handeye_calibration.hpp"

#include <cstdlib>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace easy_handeye2
{

// ============================================================================
// HandeyeCalibrationParameters
// ============================================================================

HandeyeCalibrationParameters HandeyeCalibrationParameters::fromYaml(const YAML::Node& node)
{
  HandeyeCalibrationParameters params;
  
  if (node["name"]) {
    params.name = node["name"].as<std::string>();
  }
  if (node["eye_on_hand"]) {
    params.eye_on_hand = node["eye_on_hand"].as<bool>();
  }
  if (node["robot_base_frame"]) {
    params.robot_base_frame = node["robot_base_frame"].as<std::string>();
  }
  if (node["robot_effector_frame"]) {
    params.robot_effector_frame = node["robot_effector_frame"].as<std::string>();
  }
  if (node["tracking_base_frame"]) {
    params.tracking_base_frame = node["tracking_base_frame"].as<std::string>();
  }
  if (node["tracking_marker_frame"]) {
    params.tracking_marker_frame = node["tracking_marker_frame"].as<std::string>();
  }
  
  return params;
}

YAML::Node HandeyeCalibrationParameters::toYaml() const
{
  YAML::Node node;
  node["name"] = name;
  node["eye_on_hand"] = eye_on_hand;
  node["robot_base_frame"] = robot_base_frame;
  node["robot_effector_frame"] = robot_effector_frame;
  node["tracking_base_frame"] = tracking_base_frame;
  node["tracking_marker_frame"] = tracking_marker_frame;
  return node;
}

// ============================================================================
// HandeyeCalibration
// ============================================================================

HandeyeCalibration::HandeyeCalibration()
{
  // Default identity transform
  transform_.transform.translation.x = 0.0;
  transform_.transform.translation.y = 0.0;
  transform_.transform.translation.z = 0.0;
  transform_.transform.rotation.x = 0.0;
  transform_.transform.rotation.y = 0.0;
  transform_.transform.rotation.z = 0.0;
  transform_.transform.rotation.w = 1.0;
}

HandeyeCalibration::HandeyeCalibration(
  const HandeyeCalibrationParameters& params,
  const geometry_msgs::msg::TransformStamped& transform)
: parameters_(params), transform_(transform)
{
  // Set frame IDs based on calibration type
  if (parameters_.eye_on_hand) {
    transform_.header.frame_id = parameters_.robot_effector_frame;
  } else {
    transform_.header.frame_id = parameters_.robot_base_frame;
  }
  transform_.child_frame_id = parameters_.tracking_base_frame;
}

void HandeyeCalibration::setTransform(const geometry_msgs::msg::TransformStamped& transform)
{
  transform_ = transform;
  
  // Update frame IDs
  if (parameters_.eye_on_hand) {
    transform_.header.frame_id = parameters_.robot_effector_frame;
  } else {
    transform_.header.frame_id = parameters_.robot_base_frame;
  }
  transform_.child_frame_id = parameters_.tracking_base_frame;
}

void HandeyeCalibration::setTransform(
  double tx, double ty, double tz,
  double qx, double qy, double qz, double qw)
{
  transform_.transform.translation.x = tx;
  transform_.transform.translation.y = ty;
  transform_.transform.translation.z = tz;
  transform_.transform.rotation.x = qx;
  transform_.transform.rotation.y = qy;
  transform_.transform.rotation.z = qz;
  transform_.transform.rotation.w = qw;
  
  // Update frame IDs
  if (parameters_.eye_on_hand) {
    transform_.header.frame_id = parameters_.robot_effector_frame;
  } else {
    transform_.header.frame_id = parameters_.robot_base_frame;
  }
  transform_.child_frame_id = parameters_.tracking_base_frame;
}

std::string HandeyeCalibration::getDefaultDirectory()
{
  const char* home = std::getenv("HOME");
  if (home) {
    return std::string(home) + "/.ros/easy_handeye2";
  }
  return "/tmp/easy_handeye2";
}

std::string HandeyeCalibration::getDefaultFilepath(const std::string& name)
{
  return getDefaultDirectory() + "/" + name + ".yaml";
}

bool HandeyeCalibration::saveToFile(const std::string& filepath) const
{
  // Create directory if it doesn't exist
  std::string dir = getDefaultDirectory();
  struct stat st;
  if (stat(dir.c_str(), &st) != 0) {
    mkdir(dir.c_str(), 0755);
  }
  
  std::string path = filepath.empty() ? getDefaultFilepath(parameters_.name) : filepath;
  
  try {
    YAML::Node root;
    root["parameters"] = parameters_.toYaml();
    
    YAML::Node transform_node;
    transform_node["x"] = transform_.transform.translation.x;
    transform_node["y"] = transform_.transform.translation.y;
    transform_node["z"] = transform_.transform.translation.z;
    transform_node["qx"] = transform_.transform.rotation.x;
    transform_node["qy"] = transform_.transform.rotation.y;
    transform_node["qz"] = transform_.transform.rotation.z;
    transform_node["qw"] = transform_.transform.rotation.w;
    root["transformation"] = transform_node;
    
    std::ofstream fout(path);
    fout << root;
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

std::shared_ptr<HandeyeCalibration> HandeyeCalibration::loadFromFile(const std::string& filepath)
{
  try {
    YAML::Node root = YAML::LoadFile(filepath);
    
    auto calibration = std::make_shared<HandeyeCalibration>();
    
    if (root["parameters"]) {
      calibration->parameters_ = HandeyeCalibrationParameters::fromYaml(root["parameters"]);
    }
    
    if (root["transformation"]) {
      YAML::Node t = root["transformation"];
      calibration->setTransform(
        t["x"].as<double>(),
        t["y"].as<double>(),
        t["z"].as<double>(),
        t["qx"].as<double>(),
        t["qy"].as<double>(),
        t["qz"].as<double>(),
        t["qw"].as<double>()
      );
    }
    
    return calibration;
  } catch (const std::exception& e) {
    return nullptr;
  }
}

std::string HandeyeCalibration::toYamlString() const
{
  YAML::Node root;
  root["parameters"] = parameters_.toYaml();
  
  YAML::Node transform_node;
  transform_node["x"] = transform_.transform.translation.x;
  transform_node["y"] = transform_.transform.translation.y;
  transform_node["z"] = transform_.transform.translation.z;
  transform_node["qx"] = transform_.transform.rotation.x;
  transform_node["qy"] = transform_.transform.rotation.y;
  transform_node["qz"] = transform_.transform.rotation.z;
  transform_node["qw"] = transform_.transform.rotation.w;
  root["transformation"] = transform_node;
  
  std::stringstream ss;
  ss << root;
  return ss.str();
}

std::shared_ptr<HandeyeCalibration> HandeyeCalibration::fromYamlString(const std::string& yaml_str)
{
  try {
    YAML::Node root = YAML::Load(yaml_str);
    
    auto calibration = std::make_shared<HandeyeCalibration>();
    
    if (root["parameters"]) {
      calibration->parameters_ = HandeyeCalibrationParameters::fromYaml(root["parameters"]);
    }
    
    if (root["transformation"]) {
      YAML::Node t = root["transformation"];
      calibration->setTransform(
        t["x"].as<double>(),
        t["y"].as<double>(),
        t["z"].as<double>(),
        t["qx"].as<double>(),
        t["qy"].as<double>(),
        t["qz"].as<double>(),
        t["qw"].as<double>()
      );
    }
    
    return calibration;
  } catch (const std::exception& e) {
    return nullptr;
  }
}

}  // namespace easy_handeye2
