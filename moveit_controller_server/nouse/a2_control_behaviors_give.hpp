#ifndef A2_CONTROL_BEHAVIORS_HPP__
#define A2_CONTROL_BEHAVIORS_HPP__

#include "custom_interface/srv/set_arm_action.hpp"
#include "custom_interface/srv/set_arm_joint_values.hpp"
#include "custom_interface/srv/set_arm_target_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class A2_Behavior_client : public rclcpp::Node {
  public:
    explicit A2_Behavior_client(const std::string &name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "A2_Behavior_client node is started");

        init();
        // 机械臂动作函数
        sub_action_topic_ = this->create_subscription<std_msgs::msg::String>(
            "action_topic", 1,
            std::bind(&A2_Behavior_client::robot_action_callback, this,
                      std::placeholders::_1));

        // 相机识别位姿回调函数
        sub_camera_pose_rec_ =
            this->create_subscription<geometry_msgs::msg::Point>(
                "transformed_pose", 1,
                std::bind(&A2_Behavior_client::camera_pose_rec_callback, this,
                          std::placeholders::_1));
        // hand 控制程序
        hand_control_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "hand_command_string", 1);

        // 创建一个1秒周期的定时器，绑定回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&A2_Behavior_client::timer_callback, this));

        // client初始化
        arm_target_pose_client_ =
            this->create_client<custom_interface::srv::SetArmTargetPose>(
                "set_arm_target_pose");
        arm_action_client_ =
            this->create_client<custom_interface::srv::SetArmAction>(
                "set_arm_action");
        arm_joint_values_client_ =
            this->create_client<custom_interface::srv::SetArmJointValues>(
                "set_arm_joint_values");

        // 添加发布者
        stop_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/arm/commond/stop", 1);
        hand_give_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/arm/commond", 1);
    }

    ~A2_Behavior_client() override {
        if (tf_thread_.joinable()) {
            tf_thread_.join();
        }
    }

  private:
    void init() {
        pose_receive_T = rclcpp::Time(now());
        Pub_pose_trig = false;
        Receive_trig = true;
        // 启动tf发布线程
        tf_thread_ = std::thread(&A2_Behavior_client::tf_pub_thread, this);
        arm_action_done_ = false;
        arm_action_start_ = false;
        arm_action_fail_ = false;
        action_trig = 0;
        Time_check_trig_ = false;
    }

  public:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr
        sub_camera_pose_rec_; // 相机坐标位置
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        sub_action_topic_; // 相机坐标位置
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        hand_control_publisher_; // 手部控制
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        stop_publisher_; // 用于抓到瓶子时发布 stop
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        hand_give_publisher_; // 用于松开瓶子时发布

    // client调用函数
    rclcpp::Client<custom_interface::srv::SetArmTargetPose>::SharedPtr
        arm_target_pose_client_; // 机械臂目标位姿
    rclcpp::Client<custom_interface::srv::SetArmAction>::SharedPtr
        arm_action_client_; // 机械臂动作
    rclcpp::Client<custom_interface::srv::SetArmJointValues>::SharedPtr
        arm_joint_values_client_; // 机械臂关节值

    void robot_action_callback(const std_msgs::msg::String::SharedPtr msg);
    void pose_repub_tf(const geometry_msgs::msg::Point::SharedPtr point);
    void close_handle(const std::string hand_name);
    void open_handle(const std::string hand_name);
    void
    camera_pose_rec_callback(const geometry_msgs::msg::Point::SharedPtr point);
    void tf_pub_thread();

    void timer_callback();

    // template <typename ServiceT>
    // void result_callback_(
    //     typename rclcpp::Client<ServiceT>::SharedFuture result_future) {
    //     auto response = result_future.get();
    //     RCLCPP_INFO(this->get_logger(), "计算结果：%s",
    //                 response->message.c_str());
    // }

    void set_arm_target_pose(const std::vector<double> &target_pose,
                             const std::vector<double> &offset_pose,
                             const std::string &arm_name = "left_arm");
    void set_arm_action(const std::string &action_name,
                        const std::string &arm_name = "left_arm");
    void set_arm_joint_values(const std::vector<double> &joint_values,
                              const std::string &arm_name = "left_arm");

  public:
    geometry_msgs::msg::Point receive_camera_point;
    geometry_msgs::msg::Pose detect_pose;
    geometry_msgs::msg::TransformStamped transform_stamped_pub;
    std::thread tf_thread_;

    bool Pub_pose_trig;
    bool Receive_trig;
    rclcpp::Time pose_receive_T;
    rclcpp::TimerBase::SharedPtr timer_;
    bool arm_action_done_;
    bool arm_action_start_;
    bool arm_action_fail_;
    int action_trig;
    bool Time_check_trig_;
};

#endif