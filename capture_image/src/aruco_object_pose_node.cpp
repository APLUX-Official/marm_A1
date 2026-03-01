#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class ArucoObjectPoseNode : public rclcpp::Node
{
public:
    ArucoObjectPoseNode()
        : Node("aruco_object_pose_node")
    {
        // 从ost.yaml文件读取相机内参
        if (!loadCameraParameters()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera parameters from ost.yaml");
            return;
        }

        // 初始化ArUco检测器
        initializeArucoDetector();

        // 声明压缩质量参数
        this->declare_parameter<int>("jpeg_quality", 80);
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        RCLCPP_INFO(this->get_logger(), "JPEG compression quality: %d", jpeg_quality_);

        // 创建图像发布器（用于RViz显示）
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/aruco_detection/image_raw", 10);
        
        // 创建压缩图像发布器（用于节省带宽）
        compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/aruco_detection/compressed", 10);

        // 订阅矫正后的图像
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_rect", 10,
            std::bind(&ArucoObjectPoseNode::image_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // TF监听器（用于监听 base_footprint -> object）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建 object_pose 发布器
        object_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "object_pose", 10);

        // 工作空间距离过滤参数
        this->declare_parameter<double>("ws_x_min", 0.08);
        this->declare_parameter<double>("ws_x_max", 0.4);
        this->declare_parameter<double>("ws_y_min", -0.2);
        this->declare_parameter<double>("ws_y_max", 0.2);
        this->declare_parameter<double>("ws_z_min", -0.05);
        this->declare_parameter<double>("ws_z_max", 0.1);
        ws_x_min_ = this->get_parameter("ws_x_min").as_double();
        ws_x_max_ = this->get_parameter("ws_x_max").as_double();
        ws_y_min_ = this->get_parameter("ws_y_min").as_double();
        ws_y_max_ = this->get_parameter("ws_y_max").as_double();
        ws_z_min_ = this->get_parameter("ws_z_min").as_double();
        ws_z_max_ = this->get_parameter("ws_z_max").as_double();
        RCLCPP_INFO(this->get_logger(), "Workspace filter: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]",
                    ws_x_min_, ws_x_max_, ws_y_min_, ws_y_max_, ws_z_min_, ws_z_max_);

        RCLCPP_INFO(this->get_logger(), "ArUco Object Pose Node initialized");
        RCLCPP_INFO(this->get_logger(), "ArUco marker size: %.2f mm", aruco_marker_size_);
    }

private:
    void initializeArucoDetector()
    {
        // 使用DICT_4X4_50字典 (可根据需要修改)
        aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        
        // 创建检测器参数
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        // 优化检测参数
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 10;
        detector_params_->adaptiveThreshConstant = 7;
        detector_params_->minMarkerPerimeterRate = 0.03;
        detector_params_->maxMarkerPerimeterRate = 4.0;
        detector_params_->polygonalApproxAccuracyRate = 0.03;
        detector_params_->minCornerDistanceRate = 0.05;
        detector_params_->minDistanceToBorder = 3;
        detector_params_->minMarkerDistanceRate = 0.05;
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 5;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.1;

        // ArUco标记的实际大小 (mm) - 根据您的标记大小修改
        aruco_marker_size_ = 25.0; // 50mm边长的正方形标记
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat img = cv_ptr->image;
        cv::Mat img_display = img.clone();

        // 检测ArUco标记
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;

        cv::aruco::detectMarkers(img, aruco_dictionary_, marker_corners, marker_ids, 
                                detector_params_, rejected_candidates);

        if (marker_ids.size() > 0)
        {
            // 绘制检测到的标记
            cv::aruco::drawDetectedMarkers(img_display, marker_corners, marker_ids);

            // 估计每个标记的姿态
            std::vector<cv::Vec3d> rvecs, tvecs;
            
            // 创建相机矩阵和畸变系数
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
                fx_, 0, cx_,
                0, fy_, cy_,
                0, 0, 1);
            cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F); // 假设无畸变

            cv::aruco::estimatePoseSingleMarkers(marker_corners, aruco_marker_size_ / 1000.0, 
                                               camera_matrix, dist_coeffs, rvecs, tvecs);

            // 处理第一个检测到的标记
            if (!tvecs.empty())
            {
                // 获取第一个标记的位置信息
                cv::Vec3d tvec = tvecs[0];
                cv::Vec3d rvec = rvecs[0];
                int marker_id = marker_ids[0];

                // 转换为毫米单位
                double X = tvec[0] * 1000.0; // 转换为mm
                double Y = tvec[1] * 1000.0;
                double Z = tvec[2] * 1000.0;

                // 计算标记中心点（用于显示）
                cv::Point2f center(0, 0);
                for (const auto& corner : marker_corners[0])
                {
                    center.x += corner.x;
                    center.y += corner.y;
                }
                center.x /= 4;
                center.y /= 4;

                // 输出位置信息
                RCLCPP_INFO(this->get_logger(), 
                           "ArUco Marker ID %d Position - X: %.2f mm, Y: %.2f mm, Z: %.2f mm",
                           marker_id, X, Y, Z);

                // 绘制坐标轴
                cv::aruco::drawAxis(img_display, camera_matrix, dist_coeffs, 
                                  rvec, tvec, aruco_marker_size_ / 1000.0 * 0.5);

                // 绘制标记中心点
                cv::circle(img_display, center, 5, cv::Scalar(0, 255, 0), -1);

                // 显示位置信息在图像上
                std::string pose_info = "ID:" + std::to_string(marker_id) + 
                                       " X=" + std::to_string(static_cast<int>(X)) + 
                                       "mm, Y=" + std::to_string(static_cast<int>(Y)) + 
                                       "mm, Z=" + std::to_string(static_cast<int>(Z)) + "mm";
                cv::putText(img_display, pose_info, cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                // 显示检测信息
                std::string detection_info = "ArUco Markers: " + std::to_string(marker_ids.size());
                cv::putText(img_display, detection_info, cv::Point(10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                std::string marker_size_info = "Marker Size: " + std::to_string(static_cast<int>(aruco_marker_size_)) + "mm";
                cv::putText(img_display, marker_size_info, cv::Point(10, 90), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                // 发布tf (仅位置信息)
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                t.header.frame_id = "camera_color_optical_frame"; // 相机坐标系
                t.child_frame_id = "object"; // 保持与red_object相同的frame_id

                t.transform.translation.x = X / 1000.0; // 转换为米
                t.transform.translation.y = Y / 1000.0;
                t.transform.translation.z = Z / 1000.0;

                // 无旋转 (单位四元数) - 只保留位置信息
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(t);

                // 监听 base_footprint -> object 的TF变换，发布 object_pose
                publishObjectPose(marker_id);
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "No ArUco markers detected");
            
            // 显示无检测信息
            cv::putText(img_display, "No ArUco markers detected", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }

        // 发布图像（两种格式）
        publishImages(img_display, msg->header);
    }

    void publishImages(const cv::Mat& image, const std_msgs::msg::Header& header)
    {
        // 1. 发布普通图像（用于RViz显示）
        auto image_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
        image_pub_->publish(*image_msg);

        // 2. 发布压缩图像（用于节省带宽）
        auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        compressed_msg->header = header;
        compressed_msg->format = "jpeg";

        // JPEG压缩参数
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);

        // 压缩图像
        cv::imencode(".jpg", image, compressed_msg->data, compression_params);

        // 发布压缩图像
        compressed_image_pub_->publish(std::move(compressed_msg));
    }

    // 监听 base_footprint -> object 的TF，进行工作空间过滤后发布 object_pose
    void publishObjectPose(int marker_id)
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "base_footprint", "object", tf2::TimePointZero,
                tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Could not get base_footprint->object transform: %s", ex.what());
            return;
        }

        double x_m = transform.transform.translation.x;
        double y_m = transform.transform.translation.y;
        double z_m = transform.transform.translation.z;

        // 工作空间范围过滤
        if ((x_m > ws_x_min_ && x_m < ws_x_max_) &&
            (y_m > ws_y_min_ && y_m < ws_y_max_) &&
            (z_m > ws_z_min_ && z_m < ws_z_max_)) {
            // 构建 PoseStamped 消息
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            // frame_id 使用 marker_id 字符串
            pose_msg.header.frame_id = std::to_string(marker_id);

            pose_msg.pose.position.x = x_m;
            pose_msg.pose.position.y = y_m;
            pose_msg.pose.position.z = z_m;
            pose_msg.pose.orientation = transform.transform.rotation;

            object_pose_pub_->publish(pose_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "object_pose published: frame_id=%s, pos(%.3f, %.3f, %.3f) m",
                pose_msg.header.frame_id.c_str(), x_m, y_m, z_m);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Object filtered: x=%.3f, y=%.3f, z=%.3f", x_m, y_m, z_m);
        }
    }

    bool loadCameraParameters()
    {
        try {
            std::string yaml_file_path;
            
            // 方法1: 尝试从安装的share目录读取
            try {
                std::string package_path = ament_index_cpp::get_package_share_directory("capture_image");
                yaml_file_path = package_path + "/ost.yaml";
                std::ifstream test_file(yaml_file_path);
                if (test_file.good()) {
                    test_file.close();
                    RCLCPP_INFO(this->get_logger(), "Found ost.yaml in package share directory");
                } else {
                    throw std::runtime_error("File not found in share directory");
                }
            } catch (...) {
                // 方法2: 尝试从源码目录读取
                yaml_file_path = "/home/yelume/01-program/05-mbot-marm/marm_ws/src/capture_image/ost.yaml";
                std::ifstream test_file(yaml_file_path);
                if (test_file.good()) {
                    test_file.close();
                    RCLCPP_INFO(this->get_logger(), "Found ost.yaml in source directory");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Cannot find ost.yaml file in any expected location");
                    return false;
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Loading camera parameters from: %s", yaml_file_path.c_str());
            
            // 手动解析YAML文件
            std::ifstream file(yaml_file_path);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot open camera calibration file: %s", yaml_file_path.c_str());
                return false;
            }

            std::string line;
            bool in_camera_matrix = false;
            std::vector<double> camera_data;
            
            while (std::getline(file, line)) {
                // 移除行首和行尾的空白字符
                line.erase(0, line.find_first_not_of(" \t"));
                line.erase(line.find_last_not_of(" \t") + 1);
                
                if (line.find("camera_matrix:") != std::string::npos) {
                    in_camera_matrix = true;
                    continue;
                }
                
                if (in_camera_matrix && line.find("data:") != std::string::npos) {
                    // 提取data行中的数据
                    size_t bracket_start = line.find('[');
                    if (bracket_start != std::string::npos) {
                        std::string data_line = line.substr(bracket_start + 1);
                        
                        // 继续读取直到找到闭合括号或数据结束
                        std::string full_data = data_line;
                        while (full_data.find(']') == std::string::npos && std::getline(file, line)) {
                            line.erase(0, line.find_first_not_of(" \t"));
                            line.erase(line.find_last_not_of(" \t") + 1);
                            full_data += " " + line;
                        }
                        
                        // 移除闭合括号
                        size_t bracket_end = full_data.find(']');
                        if (bracket_end != std::string::npos) {
                            full_data = full_data.substr(0, bracket_end);
                        }
                        
                        // 解析数字
                        std::stringstream ss(full_data);
                        std::string token;
                        while (std::getline(ss, token, ',')) {
                            token.erase(0, token.find_first_not_of(" \t"));
                            token.erase(token.find_last_not_of(" \t") + 1);
                            if (!token.empty()) {
                                try {
                                    double value = std::stod(token);
                                    camera_data.push_back(value);
                                } catch (const std::exception& e) {
                                    RCLCPP_WARN(this->get_logger(), "Failed to parse token: %s", token.c_str());
                                }
                            }
                        }
                        break;
                    }
                }
                
                // 如果遇到其他顶级键，退出camera_matrix解析
                if (in_camera_matrix && !line.empty() && 
                    (line.find(' ') != 0) && (line.find('\t') != 0) && 
                    line.find("rows:") == std::string::npos && 
                    line.find("cols:") == std::string::npos && 
                    line.find("data:") == std::string::npos) {
                    break;
                }
            }
            
            file.close();
            
            // 验证我们是否获得了正确数量的参数
            if (camera_data.size() != 9) {
                RCLCPP_ERROR(this->get_logger(), "Invalid camera matrix: expected 9 elements, got %zu", camera_data.size());
                return false;
            }
            
            // 提取内参 (相机矩阵是行主序存储)
            fx_ = camera_data[0];  // [0,0] 元素
            fy_ = camera_data[4];  // [1,1] 元素  
            cx_ = camera_data[2];  // [0,2] 元素
            cy_ = camera_data[5];  // [1,2] 元素

            RCLCPP_INFO(this->get_logger(), "Camera parameters loaded successfully:");
            RCLCPP_INFO(this->get_logger(), "fx: %.6f, fy: %.6f", fx_, fy_);
            RCLCPP_INFO(this->get_logger(), "cx: %.6f, cy: %.6f", cx_, cy_);

            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while loading camera parameters: %s", e.what());
            return false;
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    double fx_, fy_, cx_, cy_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_pub_;
    int jpeg_quality_;  // JPEG压缩质量 (0-100)

    // 工作空间过滤参数 (米)
    double ws_x_min_, ws_x_max_;
    double ws_y_min_, ws_y_max_;
    double ws_z_min_, ws_z_max_;
    
    // ArUco相关变量
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    double aruco_marker_size_; // ArUco标记的实际大小 (mm)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoObjectPoseNode>());
    rclcpp::shutdown();
    return 0;
}
