/**
 * @file calibration_backend_opencv.hpp
 * @brief OpenCV-based hand-eye calibration backend
 */

#ifndef EASY_HANDEYE2__CALIBRATION_BACKEND_OPENCV_HPP_
#define EASY_HANDEYE2__CALIBRATION_BACKEND_OPENCV_HPP_

#include <vector>
#include <memory>
#include <optional>

#include <yaml-cpp/yaml.h>

#include "easy_handeye2/handeye_calibration.hpp"
#include "easy_handeye2/handeye_sampler.hpp"

namespace easy_handeye2
{

/**
 * @brief Constraint parameters for limited DOF hand-eye calibration
 * 
 * When the camera has limited degrees of freedom relative to the robot
 * (e.g., 4 DOF), some transformation parameters can be fixed to known values
 * while solving for the remaining unknowns.
 */
struct CalibrationConstraints
{
  bool enabled = false;     ///< Whether constraints are enabled
  
  // Rotation constraints (in radians)
  // Use std::nullopt if the parameter should be solved
  std::optional<double> roll;    ///< Fixed roll angle (rotation around x-axis)
  std::optional<double> pitch;   ///< Fixed pitch angle (rotation around y-axis)
  std::optional<double> yaw;     ///< Fixed yaw angle (rotation around z-axis)
  
  // Translation constraints (in meters)
  // Use std::nullopt if the parameter should be solved
  std::optional<double> x;       ///< Fixed x translation
  std::optional<double> y;       ///< Fixed y translation
  std::optional<double> z;       ///< Fixed z translation
  
  /**
   * @brief Load constraints from YAML node
   */
  static CalibrationConstraints fromYaml(const YAML::Node& node);
  
  /**
   * @brief Convert constraints to YAML node  
   */
  YAML::Node toYaml() const;
  
  /**
   * @brief Get number of free (unknown) parameters
   */
  int getNumFreeParams() const;
  
  /**
   * @brief Check if this is a valid constraint configuration
   */
  bool isValid() const;
};

/**
 * @brief Parameters for constrained calibration grid search and gradient descent
 */
struct GridSearchParams
{
  // Grid search range and step
  double trans_range = 0.1;         ///< Translation search range (±meters)
  double trans_step = 0.001;        ///< Translation grid step (meters)
  double rot_range = M_PI;          ///< Rotation search range (±radians)
  double rot_step = 0.2;            ///< Rotation grid step (radians)

  // Gradient descent
  int    gd_max_iterations = 100;   ///< Max gradient descent iterations
  double gd_learning_rate = 0.001;  ///< Gradient descent learning rate
  double gd_delta = 0.0001;         ///< Numerical gradient delta

  /**
   * @brief Load grid search parameters from YAML node
   */
  static GridSearchParams fromYaml(const YAML::Node& node);
};

/**
 * @brief OpenCV-based hand-eye calibration backend
 */
class CalibrationBackendOpenCV
{
public:
  /**
   * @brief Minimum number of samples required for calibration
   */
  static constexpr size_t MIN_SAMPLES = 3;

  CalibrationBackendOpenCV() = default;

  /**
   * @brief Compute constrained hand-eye calibration (4-DOF robot)
   * @param params    Calibration frame parameters
   * @param samples   Collected calibration samples
   * @param constraints Fixed/free parameter mask from YAML
   * @param grid_params Grid search and gradient descent tuning from YAML
   * @return Calibration result, or nullptr if insufficient samples
   */
  std::shared_ptr<HandeyeCalibration> computeConstrainedCalibration(
    const HandeyeCalibrationParameters& params,
    const std::vector<CalibrationSample>& samples,
    const CalibrationConstraints& constraints,
    const GridSearchParams& grid_params = GridSearchParams{});
};

}  // namespace easy_handeye2

#endif  // EASY_HANDEYE2__CALIBRATION_BACKEND_OPENCV_HPP_
