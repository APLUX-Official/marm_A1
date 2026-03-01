#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <map>
#include <cstdlib>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// 颜色HSV阈值结构
struct ColorThreshold {
    int h_min, h_max;
    int s_min, s_max;
    int v_min, v_max;
    std::string name;
    std::string type; // "r", "g", "b"
};

class RGBDetectPoseNode : public rclcpp::Node
{
public:
    RGBDetectPoseNode()
        : Node("rgb_detect_pose_node")
    {
        // 从ost.yaml文件读取相机内参
        if (!loadCameraParameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera parameters from ost.yaml");
            return;
        }

        // 初始化颜色阈值
        initializeColorThresholds();

        // 声明参数
        this->declare_parameter<double>("circle_real_diameter_mm", 32.0); // 圆形真实直径(mm)
        this->declare_parameter<int>("jpeg_quality", 80);
        this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
        this->declare_parameter<bool>("show_gui", true);
        this->declare_parameter<std::string>("color_thresholds_file", "");
        
        circle_real_diameter_ = this->get_parameter("circle_real_diameter_mm").as_double();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        show_gui_ = this->get_parameter("show_gui").as_bool();
        thresholds_file_ = resolveThresholdsFilePath(
            this->get_parameter("color_thresholds_file").as_string());

        RCLCPP_INFO(this->get_logger(), "Circle real diameter: %.2f mm", circle_real_diameter_);
        RCLCPP_INFO(this->get_logger(), "Camera frame: %s", camera_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Color thresholds file: %s", thresholds_file_.c_str());

        // 加载颜色阈值配置（若存在）
        loadColorThresholds(thresholds_file_);

        // 创建图像发布器
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/rgb_detection/image_raw", 10);
        
        // 创建压缩图像发布器
        compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/rgb_detection/compressed", 10);

        // 创建颜色类型发布器
        color_type_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/rgb_detection/color_type", 10);

        // 订阅相机图像
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_rect", 10,
            std::bind(&RGBDetectPoseNode::image_callback, this, std::placeholders::_1));

        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // TF监听器（用于监听 base_link -> object）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建 object_pose 发布器
        object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "object_pose", 10);

        // 距离过滤参数 (mm)
        this->declare_parameter<double>("distance_min_mm", 0.08);
        this->declare_parameter<double>("distance_max_mm", 0.35);
        distance_min_mm_ = this->get_parameter("distance_min_mm").as_double();
        distance_max_mm_ = this->get_parameter("distance_max_mm").as_double();
        RCLCPP_INFO(this->get_logger(), "Distance filter: min=%.1f mm, max=%.1f mm", distance_min_mm_, distance_max_mm_);

        // 创建GUI窗口和滑动条
        if (show_gui_) {
            createTrackbars();
        }

        // 创建定时器驱动处理循环（~30fps），保证GUI事件循环不被阻塞
        gui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RGBDetectPoseNode::processingLoop, this));

        RCLCPP_INFO(this->get_logger(), "RGB Detect Pose Node initialized");
    }

    ~RGBDetectPoseNode()
    {
        if (thresholds_dirty_) {
            saveColorThresholds(thresholds_file_);
        }
        if (show_gui_) {
            cv::destroyAllWindows();
        }
    }

private:
    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double circle_real_diameter_;
    int jpeg_quality_;
    std::string camera_frame_;
    bool show_gui_;
    std::string thresholds_file_;
    bool thresholds_dirty_ = false;

    // 颜色阈值
    std::vector<ColorThreshold> color_thresholds_;
    int current_color_index_ = 0; // 当前选择的颜色索引

    // Trackbar 缓存值（避免直接绑定到 vector 内存引起崩溃）
    int tb_h_min_ = 0;
    int tb_h_max_ = 180;
    int tb_s_min_ = 0;
    int tb_s_max_ = 255;
    int tb_v_min_ = 0;
    int tb_v_max_ = 255;

    // ROS发布器和订阅器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_type_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;
    double distance_min_mm_;
    double distance_max_mm_;
    rclcpp::TimerBase::SharedPtr gui_timer_;

    // 帧缓存（回调仅存储最新帧，定时器中处理）
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std_msgs::msg::Header latest_header_;
    bool has_new_frame_ = false;

    struct DetectionResult {
        bool found = false;
        cv::Point2f center;
        float radius = 0.0f;
        cv::Point3d position_3d;
        std::string name;
        std::string type;
    };

    // 加载相机内参
    bool loadCameraParameters()
    {
        try {
            // 尝试多个可能的文件路径
            std::vector<std::string> possible_paths = {
                "/home/aidlux/marm_ws/src/capture_image/ost.yaml",
                "./ost.yaml",
                "../ost.yaml"
            };
            
            // 如果编译后的包存在，也尝试从安装目录加载
            try {
                std::string package_share_directory = 
                    ament_index_cpp::get_package_share_directory("capture_image");
                possible_paths.insert(possible_paths.begin(), 
                                     package_share_directory + "/ost.yaml");
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Package share directory not found, using fallback paths");
            }
            
            std::string yaml_file;
            bool file_found = false;
            
            // 查找存在的文件
            for (const auto& path : possible_paths) {
                std::ifstream test_file(path);
                if (test_file.good()) {
                    yaml_file = path;
                    file_found = true;
                    RCLCPP_INFO(this->get_logger(), "Found calibration file: %s", yaml_file.c_str());
                    break;
                }
            }
            
            if (!file_found) {
                RCLCPP_ERROR(this->get_logger(), "Cannot find ost.yaml in any expected location");
                return false;
            }

            // 使用 yaml-cpp 解析 ROS 格式的 YAML 文件
            YAML::Node config = YAML::LoadFile(yaml_file);
            
            // 读取 camera_matrix
            if (!config["camera_matrix"] || !config["camera_matrix"]["data"]) {
                RCLCPP_ERROR(this->get_logger(), "camera_matrix not found in YAML file");
                return false;
            }
            
            auto camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            if (camera_matrix_data.size() != 9) {
                RCLCPP_ERROR(this->get_logger(), "Invalid camera_matrix size: %zu", camera_matrix_data.size());
                return false;
            }
            
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) {
                camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix_data[i];
            }
            
            // 读取 distortion_coefficients
            if (!config["distortion_coefficients"] || !config["distortion_coefficients"]["data"]) {
                RCLCPP_ERROR(this->get_logger(), "distortion_coefficients not found in YAML file");
                return false;
            }
            
            auto dist_coeffs_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            dist_coeffs_ = cv::Mat(1, dist_coeffs_data.size(), CV_64F);
            for (size_t i = 0; i < dist_coeffs_data.size(); i++) {
                dist_coeffs_.at<double>(0, i) = dist_coeffs_data[i];
            }

            RCLCPP_INFO(this->get_logger(), "Camera parameters loaded successfully from: %s", yaml_file.c_str());
            RCLCPP_INFO(this->get_logger(), "Camera matrix: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        camera_matrix_.at<double>(0, 0),
                        camera_matrix_.at<double>(1, 1),
                        camera_matrix_.at<double>(0, 2),
                        camera_matrix_.at<double>(1, 2));
            RCLCPP_INFO(this->get_logger(), "Distortion coefficients: %.6f, %.6f, %.6f, %.6f, %.6f",
                        dist_coeffs_.at<double>(0, 0),
                        dist_coeffs_.at<double>(0, 1),
                        dist_coeffs_.at<double>(0, 2),
                        dist_coeffs_.at<double>(0, 3),
                        dist_coeffs_.at<double>(0, 4));
            
            return true;
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception loading camera parameters: %s", e.what());
            return false;
        }
    }

    // 初始化颜色阈值（默认值）
    void initializeColorThresholds()
    {
        // 红色（HSV色彩空间中，红色在0和180附近）
        color_thresholds_.push_back({0, 10, 100, 255, 100, 255, "Red", "r"});
        // color_thresholds_.push_back({170, 180, 100, 255, 100, 255, "Red2", "r"});
        
        // 绿色
        color_thresholds_.push_back({40, 80, 100, 255, 100, 255, "Green", "g"});
        
        // 蓝色
        color_thresholds_.push_back({100, 130, 100, 255, 100, 255, "Blue", "b"});
    }

    std::string resolveThresholdsFilePath(const std::string& param_path)
    {
        if (!param_path.empty()) {
            return param_path;
        }

        const char* home = std::getenv("HOME");
        if (home && std::string(home).size() > 0) {
            return std::string(home) + "/.ros/rgb_color_thresholds.yaml";
        }

        return "./rgb_color_thresholds.yaml";
    }

    void ensureParentDirectory(const std::string& file_path)
    {
        try {
            std::filesystem::path p(file_path);
            if (p.has_parent_path()) {
                std::error_code ec;
                std::filesystem::create_directories(p.parent_path(), ec);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to create parent directory: %s", e.what());
        }
    }

    bool loadColorThresholds(const std::string& file_path)
    {
        if (file_path.empty()) {
            return false;
        }

        std::ifstream test(file_path);
        if (!test.good()) {
            RCLCPP_WARN(this->get_logger(), "Color thresholds file not found: %s", file_path.c_str());
            return false;
        }

        try {
            YAML::Node config = YAML::LoadFile(file_path);
            if (!config["colors"] || !config["colors"].IsSequence()) {
                RCLCPP_WARN(this->get_logger(), "Invalid color thresholds file format: %s", file_path.c_str());
                return false;
            }

            std::vector<ColorThreshold> loaded;
            for (const auto& node : config["colors"]) {
                if (!node["type"] || !node["name"]) {
                    continue;
                }

                ColorThreshold ct;
                ct.type = node["type"].as<std::string>();
                ct.name = node["name"].as<std::string>();
                ct.h_min = node["h_min"] ? node["h_min"].as<int>() : 0;
                ct.h_max = node["h_max"] ? node["h_max"].as<int>() : 180;
                ct.s_min = node["s_min"] ? node["s_min"].as<int>() : 0;
                ct.s_max = node["s_max"] ? node["s_max"].as<int>() : 255;
                ct.v_min = node["v_min"] ? node["v_min"].as<int>() : 0;
                ct.v_max = node["v_max"] ? node["v_max"].as<int>() : 255;

                loaded.push_back(ct);
            }

            if (!loaded.empty()) {
                color_thresholds_ = loaded;
                current_color_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "Loaded %zu color thresholds from %s", loaded.size(), file_path.c_str());
                return true;
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load color thresholds: %s", e.what());
        }

        return false;
    }

    void saveColorThresholds(const std::string& file_path)
    {
        if (file_path.empty()) {
            return;
        }

        ensureParentDirectory(file_path);

        try {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "colors" << YAML::Value << YAML::BeginSeq;

            for (const auto& ct : color_thresholds_) {
                out << YAML::BeginMap;
                out << YAML::Key << "name" << YAML::Value << ct.name;
                out << YAML::Key << "type" << YAML::Value << ct.type;
                out << YAML::Key << "h_min" << YAML::Value << ct.h_min;
                out << YAML::Key << "h_max" << YAML::Value << ct.h_max;
                out << YAML::Key << "s_min" << YAML::Value << ct.s_min;
                out << YAML::Key << "s_max" << YAML::Value << ct.s_max;
                out << YAML::Key << "v_min" << YAML::Value << ct.v_min;
                out << YAML::Key << "v_max" << YAML::Value << ct.v_max;
                out << YAML::EndMap;
            }

            out << YAML::EndSeq;
            out << YAML::EndMap;

            std::ofstream fout(file_path);
            if (!fout.good()) {
                RCLCPP_WARN(this->get_logger(), "Failed to open thresholds file for writing: %s", file_path.c_str());
                return;
            }
            fout << out.c_str();
            thresholds_dirty_ = false;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to save color thresholds: %s", e.what());
        }
    }

    // 创建滑动条
    void createTrackbars()
    {
        cv::namedWindow("RGB Detection Control", cv::WINDOW_NORMAL);
        cv::resizeWindow("RGB Detection Control", 600, 400);

        if (color_thresholds_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No color thresholds available for trackbars");
            return;
        }

        if (current_color_index_ < 0 || static_cast<size_t>(current_color_index_) >= color_thresholds_.size()) {
            current_color_index_ = 0;
        }

        setTrackbarCacheFromColor(current_color_index_);

        // 颜色选择（0: Red, 1: Green, 2: Blue）
        cv::createTrackbar("Color Select", "RGB Detection Control", &current_color_index_, 
                          color_thresholds_.size() - 1);

        // HSV阈值滑动条
        cv::createTrackbar("H Min", "RGB Detection Control", 
                          &tb_h_min_, 180);
        cv::createTrackbar("H Max", "RGB Detection Control", 
                          &tb_h_max_, 180);
        cv::createTrackbar("S Min", "RGB Detection Control", 
                          &tb_s_min_, 255);
        cv::createTrackbar("S Max", "RGB Detection Control", 
                          &tb_s_max_, 255);
        cv::createTrackbar("V Min", "RGB Detection Control", 
                          &tb_v_min_, 255);
        cv::createTrackbar("V Max", "RGB Detection Control", 
                          &tb_v_max_, 255);

        RCLCPP_INFO(this->get_logger(), "Trackbars created. Use them to adjust color thresholds.");
        updateTrackbars();
    }

    void setTrackbarCacheFromColor(int color_index)
    {
        if (color_index < 0 || static_cast<size_t>(color_index) >= color_thresholds_.size()) {
            return;
        }

        const ColorThreshold& color = color_thresholds_[color_index];
        tb_h_min_ = color.h_min;
        tb_h_max_ = color.h_max;
        tb_s_min_ = color.s_min;
        tb_s_max_ = color.s_max;
        tb_v_min_ = color.v_min;
        tb_v_max_ = color.v_max;
    }

    void setTrackbarPosFromCache()
    {
        cv::setTrackbarPos("H Min", "RGB Detection Control", tb_h_min_);
        cv::setTrackbarPos("H Max", "RGB Detection Control", tb_h_max_);
        cv::setTrackbarPos("S Min", "RGB Detection Control", tb_s_min_);
        cv::setTrackbarPos("S Max", "RGB Detection Control", tb_s_max_);
        cv::setTrackbarPos("V Min", "RGB Detection Control", tb_v_min_);
        cv::setTrackbarPos("V Max", "RGB Detection Control", tb_v_max_);
    }

    // 更新滑动条的值到当前选择的颜色
    void updateTrackbars()
    {
        if (!show_gui_ || current_color_index_ >= color_thresholds_.size()) {
            return;
        }

        setTrackbarCacheFromColor(current_color_index_);
        setTrackbarPosFromCache();
    }

    // 从滑动条读取值
    void readTrackbars()
    {
        if (!show_gui_ || current_color_index_ >= color_thresholds_.size()) {
            return;
        }

        ColorThreshold& color = color_thresholds_[current_color_index_];
        int h_min = cv::getTrackbarPos("H Min", "RGB Detection Control");
        int h_max = cv::getTrackbarPos("H Max", "RGB Detection Control");
        int s_min = cv::getTrackbarPos("S Min", "RGB Detection Control");
        int s_max = cv::getTrackbarPos("S Max", "RGB Detection Control");
        int v_min = cv::getTrackbarPos("V Min", "RGB Detection Control");
        int v_max = cv::getTrackbarPos("V Max", "RGB Detection Control");

        bool changed = (color.h_min != h_min) || (color.h_max != h_max) ||
                       (color.s_min != s_min) || (color.s_max != s_max) ||
                       (color.v_min != v_min) || (color.v_max != v_max);

        if (changed) {
            color.h_min = h_min;
            color.h_max = h_max;
            color.s_min = s_min;
            color.s_max = s_max;
            color.v_min = v_min;
            color.v_max = v_max;

            thresholds_dirty_ = true;
            saveColorThresholds(thresholds_file_);
        }
    }

    std::string typeToName(const std::string& type)
    {
        if (type == "r") return "Red";
        if (type == "g") return "Green";
        if (type == "b") return "Blue";
        return "Unknown";
    }

    cv::Scalar typeToDrawColor(const std::string& type)
    {
        if (type == "r") return cv::Scalar(0, 0, 255);
        if (type == "g") return cv::Scalar(0, 255, 0);
        if (type == "b") return cv::Scalar(255, 0, 0);
        return cv::Scalar(255, 255, 255);
    }

    cv::Mat buildMaskForType(const cv::Mat& hsv_img, const std::string& type)
    {
        cv::Mat combined;
        bool has_any = false;
        for (const auto& threshold : color_thresholds_) {
            if (threshold.type != type) {
                continue;
            }

            cv::Mat mask;
            cv::inRange(hsv_img,
                        cv::Scalar(threshold.h_min, threshold.s_min, threshold.v_min),
                        cv::Scalar(threshold.h_max, threshold.s_max, threshold.v_max),
                        mask);

            if (!has_any) {
                combined = mask;
                has_any = true;
            } else {
                cv::bitwise_or(combined, mask, combined);
            }
        }

        return combined;
    }

    DetectionResult detectColorType(const cv::Mat& hsv_img, const std::string& type)
    {
        DetectionResult result;
        result.type = type;
        result.name = typeToName(type);

        cv::Mat mask = buildMaskForType(hsv_img, type);
        if (mask.empty()) {
            return result;
        }

        // 形态学操作去噪
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_area = 0.0;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 100) continue; // 过滤小面积

            // 拟合圆
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            // 验证是否为圆形（使用圆度）
            double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 0.0) continue;
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);

            if (circularity > 0.7 && radius > 10) {
                if (area > best_area) {
                    best_area = area;
                    result.found = true;
                    result.center = center;
                    result.radius = radius;
                }
            }
        }

        if (result.found) {
            result.position_3d = calculate3DPosition(result.center, result.radius * 2);
        }

        return result;
    }

    // 图像回调函数：仅缓存最新帧，避免阻塞
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = cv_ptr->image;
        latest_header_ = msg->header;
        has_new_frame_ = true;
    }

    // 定时器驱动的处理循环（~30fps），保证GUI不卡顿
    void processingLoop()
    {
        cv::Mat img;
        std_msgs::msg::Header header;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!has_new_frame_) {
                // 没有新帧，仍需处理GUI事件
                if (show_gui_) {
                    cv::waitKey(1);
                }
                return;
            }
            img = latest_frame_.clone();
            header = latest_header_;
            has_new_frame_ = false;
        }

        cv::Mat img_display = img.clone();

        // 更新滑动条值
        if (show_gui_) {
            static int last_color_index = -1;
            if (current_color_index_ != last_color_index) {
                updateTrackbars();
                last_color_index = current_color_index_;
            }
            readTrackbars();
        }

        // 转换到HSV色彩空间
        cv::Mat hsv_img;
        cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

        // 对每种颜色进行检测（同时检测）
        std::map<std::string, DetectionResult> detections;
        detections["r"] = detectColorType(hsv_img, "r");
        detections["g"] = detectColorType(hsv_img, "g");
        detections["b"] = detectColorType(hsv_img, "b");

        // 绘制所有检测到的圆形
        for (const auto& kv : detections) {
            const DetectionResult& det = kv.second;
            if (!det.found) {
                continue;
            }

            cv::Scalar draw_color = typeToDrawColor(det.type);
            cv::circle(img_display, det.center, static_cast<int>(det.radius), draw_color, 2);
            cv::circle(img_display, det.center, 2, cv::Scalar(0, 0, 255), -1);

            std::string info = det.name + " (" + det.type + ")";
            cv::putText(img_display, info,
                        cv::Point(det.center.x - 50, det.center.y - det.radius - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2);

            std::string pos_info = cv::format("X:%.1f Y:%.1f Z:%.1f mm",
                                              det.position_3d.x, det.position_3d.y, det.position_3d.z);
            cv::putText(img_display, pos_info,
                        cv::Point(det.center.x - 50, det.center.y + det.radius + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
        }

        // 发布优先级：r -> g -> b
        DetectionResult* publish_det = nullptr;
        if (detections["r"].found) {
            publish_det = &detections["r"];
        } else if (detections["g"].found) {
            publish_det = &detections["g"];
        } else if (detections["b"].found) {
            publish_det = &detections["b"];
        }

        if (publish_det) {
            publishTF(publish_det->position_3d, publish_det->type);

            auto color_msg = std_msgs::msg::String();
            color_msg.data = publish_det->type;
            color_type_pub_->publish(color_msg);

            // 监听 base_link -> object 的TF变换，并发布 object_pose
            publishObjectPose(publish_det->type);

            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //                     "Publishing %s | pos (%.1f, %.1f, %.1f) mm | detected: R=%s G=%s B=%s",
            //                     publish_det->name.c_str(),
            //                     publish_det->position_3d.x, publish_det->position_3d.y, publish_det->position_3d.z,
            //                     detections["r"].found ? "Y" : "N",
            //                     detections["g"].found ? "Y" : "N",
            //                     detections["b"].found ? "Y" : "N");
        }

        // 显示当前选择的颜色和检测状态
        if (show_gui_) {
            int y_offset = 30;
            if (current_color_index_ < color_thresholds_.size()) {
                std::string current_color_info = "Editing: " + 
                    color_thresholds_[current_color_index_].name +
                    " (" + color_thresholds_[current_color_index_].type + ")";
                cv::putText(img_display, current_color_info,
                           cv::Point(10, y_offset),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                y_offset += 30;
            }

            // 显示各颜色检测状态
            std::string status = cv::format("R:%s  G:%s  B:%s  | Pub: %s",
                detections["r"].found ? "ON" : "--",
                detections["g"].found ? "ON" : "--",
                detections["b"].found ? "ON" : "--",
                publish_det ? publish_det->type.c_str() : "NONE");
            cv::putText(img_display, status,
                       cv::Point(10, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }

        // 构建当前编辑颜色的掩码预览窗口
        if (show_gui_ && current_color_index_ < color_thresholds_.size()) {
            // 获取当前编辑颜色的类型
            const std::string& edit_type = color_thresholds_[current_color_index_].type;
            
            // 构建该类型的合并掩码
            cv::Mat edit_mask = buildMaskForType(hsv_img, edit_type);
            if (!edit_mask.empty()) {
                // 形态学去噪（与检测一致）
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
                cv::morphologyEx(edit_mask, edit_mask, cv::MORPH_OPEN, kernel);
                cv::morphologyEx(edit_mask, edit_mask, cv::MORPH_CLOSE, kernel);

                // 提取边缘
                cv::Mat edges;
                cv::Canny(edit_mask, edges, 50, 150);

                // 将边缘叠加到原图上（绿色高亮）
                cv::Mat mask_overlay = img_display.clone();
                mask_overlay.setTo(cv::Scalar(0, 255, 0), edges);

                // 构建掩码彩色预览：原图中被掩码选中的区域正常显示，其余变暗
                cv::Mat mask_color_preview;
                img.copyTo(mask_color_preview);
                cv::Mat dark;
                mask_color_preview.convertTo(dark, -1, 0.3, 0); // 整体变暗
                img.copyTo(dark, edit_mask);                      // 掩码区域保留原色
                
                // 在暗图上叠加边缘轮廓
                dark.setTo(typeToDrawColor(edit_type), edges);

                // 在预览图左上角标注当前阈值
                const ColorThreshold& ct = color_thresholds_[current_color_index_];
                std::string th_info = cv::format("H[%d-%d] S[%d-%d] V[%d-%d]",
                    ct.h_min, ct.h_max, ct.s_min, ct.s_max, ct.v_min, ct.v_max);
                cv::putText(dark, ct.name + " (" + ct.type + ") " + th_info,
                           cv::Point(10, 25),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                // 显示掩码预览窗口
                cv::imshow("Mask Preview (current threshold)", dark);
            }
        }

        // 显示图像（waitKey给GUI足够时间响应）
        if (show_gui_) {
            cv::imshow("RGB Detection Result", img_display);
            cv::waitKey(1);
        }

        // 发布图像
        publishImages(img_display, header);
    }

    // 计算3D位置
    cv::Point3d calculate3DPosition(const cv::Point2f& center, double diameter_pixels)
    {
        // 从相机内参获取焦距
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);
        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);

        // 计算Z坐标（深度）
        // 根据公式: diameter_pixels = (fx * real_diameter) / Z
        double Z = (fx * circle_real_diameter_) / diameter_pixels;

        // 计算X和Y坐标
        double X = (center.x - cx) * Z / fx;
        double Y = (center.y - cy) * Z / fy;

        return cv::Point3d(X, Y, Z);
    }

    // 根据颜色类型返回 frame_id 编号字符串: r->1, g->2, b->3
    std::string colorTypeToFrameId(const std::string& type)
    {
        if (type == "r") return "1";
        if (type == "g") return "2";
        if (type == "b") return "3";
        return "0";
    }

    // 监听 base_link -> object 的TF，进行距离过滤后发布 object_pose
    void publishObjectPose(const std::string& color_type)
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "base_footprint", "object", tf2::TimePointZero,
                tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Could not get base_link->object transform: %s", ex.what());
            return;
        }

        // 计算欧氏距离 (m -> mm)
        double x_m = transform.transform.translation.x;
        double y_m = transform.transform.translation.y;
        double z_m = transform.transform.translation.z;
        // double distance_mm = std::sqrt(x_m * x_m + y_m * y_m + z_m * z_m);

        if((x_m>0.1&&x_m<0.4)&&(y_m>-0.2&&y_m<0.2)&&(z_m>-0.05&&z_m<0.1)){
            // 构建 PoseStamped 消息
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            // frame_id 利用识别的颜色类型区分: r->1, g->2, b->3
            pose_msg.header.frame_id = colorTypeToFrameId(color_type);

            pose_msg.pose.position.x = x_m;
            pose_msg.pose.position.y = y_m;
            pose_msg.pose.position.z = z_m;
            pose_msg.pose.orientation = transform.transform.rotation;

            object_pose_pub_->publish(pose_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "object_pose published: frame_id=%s, pos(%.3f, %.3f, %.3f) m",
            pose_msg.header.frame_id.c_str(), x_m, y_m, z_m);
        }else{
        
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Object filtered: x_m: %f,y_m: %f,z_m: %f", x_m, y_m, z_m);
            return;
        }



    }

    // 发布TF
    void publishTF(const cv::Point3d& position, const std::string& color_type)
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->now();
        transform.header.frame_id = camera_frame_;
        transform.child_frame_id = "object";

        // 位置 (mm转换为m)
        transform.transform.translation.x = position.x / 1000.0;
        transform.transform.translation.y = position.y / 1000.0;
        transform.transform.translation.z = position.z / 1000.0;

        // 旋转（单位四元数，无旋转）
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    // 发布图像
    void publishImages(const cv::Mat& img, const std_msgs::msg::Header& header)
    {
        // // 发布原始图像
        // auto img_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        // image_pub_->publish(*img_msg);

        // 发布压缩图像
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        std::vector<uchar> buf;
        cv::imencode(".jpg", img, buf, params);

        auto compressed_msg = sensor_msgs::msg::CompressedImage();
        compressed_msg.header = header;
        compressed_msg.format = "jpeg";
        compressed_msg.data = buf;
        compressed_image_pub_->publish(compressed_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDetectPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
