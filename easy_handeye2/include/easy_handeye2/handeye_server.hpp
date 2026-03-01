/**
 * @file handeye_server.hpp
 * @brief Hand-eye calibration server node (simplified)
 * 
 * Only provides core calibration services:
 *   - TakeSample: collect a calibration sample from TF
 *   - ComputeCalibration: compute the hand-eye calibration
 *   - SaveCalibration: save the calibration result to file
 * 
 * All parameters (frames, constraints, grid search) are configured via YAML file.
 */

#ifndef EASY_HANDEYE2__HANDEYE_SERVER_HPP_
#define EASY_HANDEYE2__HANDEYE_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "easy_handeye2/handeye_calibration.hpp"
#include "easy_handeye2/handeye_sampler.hpp"
#include "easy_handeye2/calibration_backend_opencv.hpp"
// GridSearchParams is defined in calibration_backend_opencv.hpp

// Service messages (only calibration-related)
#include <easy_handeye2/srv/take_sample.hpp>
#include <easy_handeye2/srv/compute_calibration.hpp>

namespace easy_handeye2
{

/**
 * @brief ROS2 node providing hand-eye calibration services
 * 
 * Simplified version: all parameters are loaded from YAML config file.
 * Only core calibration services are provided.
 */
class HandeyeServer : public rclcpp::Node
{
public:
  explicit HandeyeServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HandeyeServer() = default;

private:
  /**
   * @brief Load all configuration from YAML file
   */
  void loadConfigFromYaml(const std::string& config_file);

  /**
   * @brief Initialize ROS services
   */
  void initializeServices();

  // Service callbacks (core calibration only)
  void takeSampleCallback(
    const std::shared_ptr<easy_handeye2::srv::TakeSample::Request> request,
    std::shared_ptr<easy_handeye2::srv::TakeSample::Response> response);

  void computeCalibrationCallback(
    const std::shared_ptr<easy_handeye2::srv::ComputeCalibration::Request> request,
    std::shared_ptr<easy_handeye2::srv::ComputeCalibration::Response> response);

  // Member variables
  HandeyeCalibrationParameters params_;
  CalibrationConstraints constraints_;
  GridSearchParams grid_search_params_;
  
  std::shared_ptr<CalibrationBackendOpenCV> calibration_backend_;
  std::shared_ptr<HandeyeSampler> sampler_;
  std::shared_ptr<HandeyeCalibration> last_calibration_;

  // ROS services (core calibration only)
  rclcpp::Service<easy_handeye2::srv::TakeSample>::SharedPtr take_sample_srv_;
  rclcpp::Service<easy_handeye2::srv::ComputeCalibration>::SharedPtr compute_calibration_srv_;
};

}  // namespace easy_handeye2

#endif  // EASY_HANDEYE2__HANDEYE_SERVER_HPP_
