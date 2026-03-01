/******************************************************************************
 * Copyright 2020-2025, zhangsai. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <vector>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace capture_image {

class CaptureImage : public rclcpp::Node {
public:
    explicit CaptureImage(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~CaptureImage();

private:
    void init_data();
    void run_spin();
    bool load_calibration_from_yaml(const std::string& file_path);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_raw_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_rect_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_image_info_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int output_rate_;
    std::string video_port_;
    std::string calibration_file_;
    bool camera_init_;
    int image_height_, image_width_;
    std::vector<double> ext_K_, ext_D_, ext_R_, ext_P_;
    bool pub_tf_;
    int frame_width_, frame_height_;
    sensor_msgs::msg::CameraInfo camera_info_;
};
} // namespace capture_image