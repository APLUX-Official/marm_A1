/**
 * @file handeye_sampler.hpp
 * @brief TF sampler for hand-eye calibration
 */

#ifndef EASY_HANDEYE2__HANDEYE_SAMPLER_HPP_
#define EASY_HANDEYE2__HANDEYE_SAMPLER_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "easy_handeye2/handeye_calibration.hpp"

namespace easy_handeye2
{

/**
 * @brief A single calibration sample containing robot and optical transforms
 */
struct CalibrationSample
{
  geometry_msgs::msg::TransformStamped robot_transform;   ///< Robot transform (base-effector)
  geometry_msgs::msg::TransformStamped optical_transform; ///< Optical transform (camera-marker)
  rclcpp::Time timestamp;
};

/**
 * @brief Manages TF sampling for hand-eye calibration
 */
class HandeyeSampler
{
public:
  /**
   * @brief Constructor
   * @param node ROS2 node for TF operations
   * @param params Calibration parameters
   */
  HandeyeSampler(
    rclcpp::Node* node,
    const HandeyeCalibrationParameters& params);
  
  /**
   * @brief Destructor
   */
  ~HandeyeSampler() = default;
  
  /**
   * @brief Take a new sample from TF
   * @return true if sample was successfully taken
   */
  bool takeSample();
  
  /**
   * @brief Remove a sample by index
   * @param index Sample index to remove
   * @return true if successful
   */
  bool removeSample(size_t index);
  
  /**
   * @brief Remove the last sample
   * @return true if successful
   */
  bool removeLastSample();
  
  /**
   * @brief Clear all samples
   */
  void clearSamples();
  
  /**
   * @brief Get all collected samples
   */
  const std::vector<CalibrationSample>& getSamples() const { return samples_; }
  
  /**
   * @brief Get number of samples
   */
  size_t getSampleCount() const { return samples_.size(); }
  
  /**
   * @brief Check if required TF frames are available
   * @param timeout Timeout duration
   * @return true if frames are available
   */
  bool waitForFrames(const std::chrono::seconds& timeout = std::chrono::seconds(10));
  
  /**
   * @brief Update calibration parameters
   */
  void setParameters(const HandeyeCalibrationParameters& params) { params_ = params; }
  
  /**
   * @brief Get current parameters
   */
  const HandeyeCalibrationParameters& getParameters() const { return params_; }

  /**
   * @brief Save all samples to YAML file
   * @param filepath Path to save file (default: ~/.ros/easy_handeye2/samples_<name>.yaml)
   * @return true if successful
   */
  bool saveSamplesToFile(const std::string& filepath = "") const;
  
  /**
   * @brief Load samples from YAML file
   * @param filepath Path to YAML file
   * @return true if successful
   */
  bool loadSamplesFromFile(const std::string& filepath = "");
  
  /**
   * @brief Get default samples file path
   * @param name Calibration name
   * @return Default file path
   */
  static std::string getDefaultSamplesFilepath(const std::string& name);
  
  /**
   * @brief Add a sample directly (used when loading from file)
   * @param sample Sample to add
   */
  void addSample(const CalibrationSample& sample) { samples_.push_back(sample); }

private:
  /**
   * @brief Get current transforms from TF
   * @param sample Output sample
   * @return true if successful
   */
  bool getCurrentTransforms(CalibrationSample& sample);
  
  rclcpp::Node* node_;
  HandeyeCalibrationParameters params_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::vector<CalibrationSample> samples_;
};

}  // namespace easy_handeye2

#endif  // EASY_HANDEYE2__HANDEYE_SAMPLER_HPP_
