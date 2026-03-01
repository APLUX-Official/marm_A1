/**
 * @file handeye_calibration.hpp
 * @brief Hand-eye calibration data structures and utilities
 */

#ifndef EASY_HANDEYE2__HANDEYE_CALIBRATION_HPP_
#define EASY_HANDEYE2__HANDEYE_CALIBRATION_HPP_

#include <string>
#include <memory>
#include <fstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <yaml-cpp/yaml.h>

namespace easy_handeye2
{

/**
 * @brief Parameters for hand-eye calibration
 */
struct HandeyeCalibrationParameters
{
  std::string name;                    ///< Calibration name/namespace
  bool eye_on_hand;                    ///< true for eye-in-hand, false for eye-on-base
  std::string robot_base_frame;        ///< Robot base frame name
  std::string robot_effector_frame;    ///< Robot end-effector frame name
  std::string tracking_base_frame;     ///< Camera/tracking system frame name
  std::string tracking_marker_frame;   ///< Marker frame name
  
  /**
   * @brief Load parameters from YAML node
   */
  static HandeyeCalibrationParameters fromYaml(const YAML::Node& node);
  
  /**
   * @brief Convert parameters to YAML node
   */
  YAML::Node toYaml() const;
};

/**
 * @brief Hand-eye calibration result
 */
class HandeyeCalibration
{
public:
  /**
   * @brief Default constructor
   */
  HandeyeCalibration();
  
  /**
   * @brief Constructor with parameters and transformation
   * @param params Calibration parameters
   * @param transform The calibration transform
   */
  HandeyeCalibration(
    const HandeyeCalibrationParameters& params,
    const geometry_msgs::msg::TransformStamped& transform);
  
  /**
   * @brief Get calibration parameters
   */
  const HandeyeCalibrationParameters& getParameters() const { return parameters_; }
  
  /**
   * @brief Set calibration parameters
   */
  void setParameters(const HandeyeCalibrationParameters& params) { parameters_ = params; }
  
  /**
   * @brief Get the calibration transform
   */
  const geometry_msgs::msg::TransformStamped& getTransform() const { return transform_; }
  
  /**
   * @brief Set the calibration transform
   */
  void setTransform(const geometry_msgs::msg::TransformStamped& transform);
  
  /**
   * @brief Set transform from translation and quaternion
   */
  void setTransform(
    double tx, double ty, double tz,
    double qx, double qy, double qz, double qw);
  
  /**
   * @brief Save calibration to YAML file
   * @param filepath Path to save file
   * @return true if successful
   */
  bool saveToFile(const std::string& filepath) const;
  
  /**
   * @brief Load calibration from YAML file
   * @param filepath Path to load file
   * @return Loaded calibration or nullptr if failed
   */
  static std::shared_ptr<HandeyeCalibration> loadFromFile(const std::string& filepath);
  
  /**
   * @brief Convert to YAML string
   */
  std::string toYamlString() const;
  
  /**
   * @brief Parse from YAML string
   */
  static std::shared_ptr<HandeyeCalibration> fromYamlString(const std::string& yaml_str);
  
  /**
   * @brief Get default calibration directory
   */
  static std::string getDefaultDirectory();
  
  /**
   * @brief Get default filepath for a calibration name
   */
  static std::string getDefaultFilepath(const std::string& name);

private:
  HandeyeCalibrationParameters parameters_;
  geometry_msgs::msg::TransformStamped transform_;
};

}  // namespace easy_handeye2

#endif  // EASY_HANDEYE2__HANDEYE_CALIBRATION_HPP_
