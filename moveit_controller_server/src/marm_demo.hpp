#ifndef MARM_DEMO_HPP__
#define MARM_DEMO_HPP__

#include "custom_interface/srv/set_arm_action.hpp"
#include "custom_interface/srv/set_arm_target_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/parameter_client.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include "std_msgs/msg/bool.hpp"

class A2_Behavior_client : public rclcpp::Node {
public:
    explicit A2_Behavior_client(const std::string &name);
    ~A2_Behavior_client() override {}

private:
    void init() {
        arm_action_done_ = false;
        arm_action_start_ = false;
        arm_action_fail_ = false;
        action_trig = 0;
        planning_retry_count_ = 0;
        grasp_retry_count_ = 0;
        current_grasp_x_offset_ = 0.0;
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void timer_callback();
    void set_arm_target_pose(const std::vector<double> &target_pose,
                             const std::vector<double> &offset_pose,
                             const std::string &arm_name = "arm");
    void set_arm_action(const std::string &action_name,
                        const std::string &arm_name = "arm");
    
    // 使用MoveIt选择合适的pitch角度
    bool find_optimal_pitch(double x, double y, double z, double roll, double yaw, 
                           geometry_msgs::msg::Pose &optimal_pose);
    
    // 发布目标抓取位姿的TF用于可视化
    void publish_target_pose_tf(const geometry_msgs::msg::Pose &pose, const std::string &frame_id = "target_grasp_pose");
    
    // 加载运动学参数
    void load_kinematics_parameters();
    
    // 加载默认运动学参数（回退方案）
    void load_default_kinematics_parameters();
    
    // 夹爪控制函数
    void control_gripper(bool close_gripper);
    
    // object_pose 话题回调
    void object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // 检测物体是否还存在（用于验证抓取是否成功）
    // target_frame_id: 颜色类型过滤 (1=r, 2=g, 3=b)，空串不过滤
    bool is_object_still_detected(double timeout_seconds = 2.0, const std::string& target_frame_id = "");

public:
    rclcpp::Client<custom_interface::srv::SetArmTargetPose>::SharedPtr arm_target_pose_client_;
    rclcpp::Client<custom_interface::srv::SetArmAction>::SharedPtr arm_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 状态控制变量
    int action_trig;
    bool arm_action_done_;
    bool arm_action_start_;
    bool arm_action_fail_;
    
    // 重试和超时控制
    int planning_retry_count_;
    static const int MAX_PLANNING_RETRIES = 10;
    std::chrono::steady_clock::time_point planning_start_time_;
    static const int PLANNING_TIMEOUT_SECONDS = 30;
    
    // 物体位姿
    geometry_msgs::msg::Pose target_pose;
    
    // MoveIt接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    
    // MoveIt规划参数（从YAML文件加载）
    double velocity_scaling_factor_;
    double acceleration_scaling_factor_;
    double planning_time_;
    int planning_attempts_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;
    
    // 抓取位置微调参数（从YAML文件加载）
    double grasp_offset_x_;
    double grasp_offset_y_;
    double grasp_offset_z_;
    
    // 抓取重试参数（从YAML文件加载）
    bool grasp_retry_enabled_;
    int grasp_max_retries_;
    double grasp_x_adjust_step_;
    double grasp_detection_timeout_;
    
    // 抓取重试状态
    int grasp_retry_count_;
    double current_grasp_x_offset_;  // 当前抓取的额外X偏移（用于重试）
    double current_grasp_z_offset_;  // 当前抓取的额外Z偏移（用于重试）
    
    // 夹爪控制发布器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_publisher_;

    // object_pose 订阅相关
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
    std::mutex object_pose_mutex_;
    geometry_msgs::msg::PoseStamped latest_object_pose_;
    bool has_object_pose_ = false;
    std::string grasped_frame_id_;  // 记录当前抓取目标的 frame_id (颜色编号: 1=r, 2=g, 3=b)
};

#endif




