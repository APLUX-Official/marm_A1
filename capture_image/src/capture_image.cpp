/******************************************************************************
 * Copyright 2020-2025, zhangsai. All Rights Reserved.
 *****************************************************************************/

#include "capture_image.hpp"

namespace capture_image {

CaptureImage::CaptureImage(const rclcpp::NodeOptions &options)
    : Node("capture_image", options)
{
    this->declare_parameter("image_raw_topic", "/camera/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera_info");
    this->declare_parameter("image_rect_topic", "/camera/image_rect");
    this->declare_parameter("pub_tf", true);
    this->declare_parameter("output_rate", 10);
    this->declare_parameter("video_port", "/dev/video0");
    this->declare_parameter("calibration_file", "ost.yaml");

    std::string image_raw_topic = this->get_parameter("image_raw_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string image_rect_topic = this->get_parameter("image_rect_topic").as_string();
    pub_tf_ = this->get_parameter("pub_tf").as_bool();
    output_rate_ = this->get_parameter("output_rate").as_int();
    video_port_ = this->get_parameter("video_port").as_string();
    calibration_file_ = this->get_parameter("calibration_file").as_string();

    pub_image_raw_ = this->create_publisher<sensor_msgs::msg::Image>(image_raw_topic, 10);
    pub_image_rect_ = this->create_publisher<sensor_msgs::msg::Image>(image_rect_topic, 10);
    pub_image_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);


    std::cout<<"image_raw_topic_:"<<image_raw_topic << std::endl;
    std::cout<<"camera_info_topic_:"<<camera_info_topic << std::endl;
    std::cout<<"image_rect_topic_:"<<image_rect_topic << std::endl;


    init_data();
    std::thread(&CaptureImage::run_spin, this).detach();
}

CaptureImage::~CaptureImage() {}

void CaptureImage::init_data()
{
    camera_init_ = false;
    
    // Load calibration parameters from YAML file
    if (!load_calibration_from_yaml(calibration_file_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load calibration from file: %s", calibration_file_.c_str());
        return;
    }

    camera_info_.d = ext_D_;
    for (size_t i = 0; i < 9 && i < ext_K_.size(); ++i)
        camera_info_.k[i] = ext_K_[i];
    for (size_t i = 0; i < 9 && i < ext_R_.size(); ++i)
        camera_info_.r[i] = ext_R_[i];
    for (size_t i = 0; i < 12 && i < ext_P_.size(); ++i)
        camera_info_.p[i] = ext_P_[i];
}

void CaptureImage::run_spin()
{
    cv::VideoCapture capture(video_port_);
    cv::Mat map1, map2;
    rclcpp::Rate loop(output_rate_);

    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_FPS, output_rate_);

    while (rclcpp::ok())
    {
        cv::Mat frame;
        if (capture.isOpened())
        {
            if (!capture.read(frame))
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "read camera frame fail!");
            }
            else
            {
                image_height_ = frame.rows;
                image_width_ = frame.cols;
                if (!camera_init_ && ext_K_.size() == 9 && ext_D_.size() == 5)
                {
                    cv::Mat K = (cv::Mat_<double>(3, 3) << ext_K_[0], ext_K_[1], ext_K_[2], ext_K_[3], ext_K_[4], ext_K_[5], ext_K_[6], ext_K_[7], ext_K_[8]);
                    cv::Mat D = (cv::Mat_<double>(5, 1) << ext_D_[0], ext_D_[1], ext_D_[2], ext_D_[3], ext_D_[4]);
                    cv::Size imageSize(image_width_, image_height_);
                    double alpha = 0;
                    cv::Mat NewCameraMatrix = cv::getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);
                    cv::initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, map1, map2);
                    for (int i = 0; i < 9; ++i)
                        camera_info_.k[i] = NewCameraMatrix.at<double>(i / 3, i % 3);
                    camera_init_ = true;
                }

                std_msgs::msg::Header header;
                header.stamp = this->now();
                header.frame_id = "camera_color_optical_frame";

                if (pub_image_raw_->get_subscription_count() > 0)
                {
                    cv_bridge::CvImage out_msg(header, sensor_msgs::image_encodings::BGR8, frame);
                    pub_image_raw_->publish(*out_msg.toImageMsg());
                }
                if (pub_image_rect_->get_subscription_count() > 0)
                {
                    cv::Mat UndistortImage;
                    cv::remap(frame, UndistortImage, map1, map2, cv::INTER_LINEAR);
                    cv_bridge::CvImage out_msg(header, sensor_msgs::image_encodings::BGR8, UndistortImage);
                    pub_image_rect_->publish(*out_msg.toImageMsg());
                }
                if (pub_image_info_->get_subscription_count() > 0)
                {
                    camera_info_.header = header;
                    camera_info_.height = image_height_;
                    camera_info_.width = image_width_;
                    camera_info_.distortion_model = "plumb_bob";
                    pub_image_info_->publish(camera_info_);
                }
                if (pub_tf_)
                {
                    geometry_msgs::msg::TransformStamped transformStamped;
                    transformStamped.header = header;
                    transformStamped.header.frame_id = "camera_link";
                    transformStamped.child_frame_id = "camera_color_optical_frame";
                    transformStamped.transform.translation.x = 0.0;
                    transformStamped.transform.translation.y = 0.0;
                    transformStamped.transform.translation.z = 0.0;
                    transformStamped.transform.rotation.x = -0.5;
                    transformStamped.transform.rotation.y = 0.5;
                    transformStamped.transform.rotation.z = -0.5;
                    transformStamped.transform.rotation.w = 0.5;
                    tf_broadcaster_->sendTransform(transformStamped);
                }
            }
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "camera open fail!");
        }
        loop.sleep();
    }
}

bool CaptureImage::load_calibration_from_yaml(const std::string& file_path)
{
    try {
        std::string full_path;
        
        // Check if path is absolute or relative
        if (file_path[0] == '/') {
            full_path = file_path;
        } else {
            // Relative path, construct full path using package share directory
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("capture_image");
            full_path = package_share_directory + "/" + file_path;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loading calibration from: %s", full_path.c_str());
        
        YAML::Node config = YAML::LoadFile(full_path);
        
        // Load camera matrix (K)
        if (config["camera_matrix"]) {
            auto camera_matrix = config["camera_matrix"]["data"];
            ext_K_.clear();
            for (size_t i = 0; i < camera_matrix.size(); ++i) {
                ext_K_.push_back(camera_matrix[i].as<double>());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "camera_matrix not found in calibration file");
            return false;
        }
        
        // Load distortion coefficients (D)
        if (config["distortion_coefficients"]) {
            auto distortion_coeffs = config["distortion_coefficients"]["data"];
            ext_D_.clear();
            for (size_t i = 0; i < distortion_coeffs.size(); ++i) {
                ext_D_.push_back(distortion_coeffs[i].as<double>());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "distortion_coefficients not found in calibration file");
            return false;
        }
        
        // Load rectification matrix (R)
        if (config["rectification_matrix"]) {
            auto rectification_matrix = config["rectification_matrix"]["data"];
            ext_R_.clear();
            for (size_t i = 0; i < rectification_matrix.size(); ++i) {
                ext_R_.push_back(rectification_matrix[i].as<double>());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "rectification_matrix not found in calibration file");
            return false;
        }
        
        // Load projection matrix (P)
        if (config["projection_matrix"]) {
            auto projection_matrix = config["projection_matrix"]["data"];
            ext_P_.clear();
            for (size_t i = 0; i < projection_matrix.size(); ++i) {
                ext_P_.push_back(projection_matrix[i].as<double>());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "projection_matrix not found in calibration file");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration from: %s", full_path.c_str());
        return true;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading calibration file: %s", e.what());
        return false;
    }
}

} // namespace capture_image