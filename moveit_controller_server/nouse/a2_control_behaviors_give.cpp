#include "a2_control_behaviors.hpp"
#include <atomic>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/**
 * 将识别的物体坐标转换为机械臂坐标，并通过tf发布出来
 */
void A2_Behavior_client::pose_repub_tf(
    const geometry_msgs::msg::Point::SharedPtr point) {
    // 创建 tf2 消息
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "rgbd_head_camera_color_optical_frame";
    transform_stamped.child_frame_id = "detected_object";
    transform_stamped.transform.translation.x = point->x;
    transform_stamped.transform.translation.y = point->y;
    transform_stamped.transform.translation.z = point->z;
    // 假设没有旋转
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    // 发布tf
    static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    if (!tf_broadcaster) {
        tf_broadcaster =
            std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    }
    tf_broadcaster->sendTransform(transform_stamped);

    // 监听tf并转换到arm_base_link坐标系下
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    if (!tf_buffer) {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    try {
        // 查询detected_object到arm_base_link的变换
        geometry_msgs::msg::TransformStamped transform_to_base =
            tf_buffer->lookupTransform("arm_base_link", "detected_object",
                                       tf2::TimePointZero);

        // 将点从detected_object坐标系下(原点)变换到arm_base_link下
        detect_pose.position.x = transform_to_base.transform.translation.x;
        detect_pose.position.y = transform_to_base.transform.translation.y;
        detect_pose.position.z = transform_to_base.transform.translation.z;
        detect_pose.orientation.x = 0.0;
        detect_pose.orientation.y = 0.0;
        detect_pose.orientation.z = 0.0;
        detect_pose.orientation.w = 1.0;

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform detected_object to arm_base_link: %s",
                    ex.what());
    }
}

// 线程函数，用于持续发布tf
void A2_Behavior_client::tf_pub_thread() {
    std::cout << "start thread..." << std::endl;
    rclcpp::Rate rate(10); // 10Hz
    while (rclcpp::ok()) {
        if (this->Pub_pose_trig) {
            auto point_ptr = std::make_shared<geometry_msgs::msg::Point>(
                this->receive_camera_point);
            pose_repub_tf(point_ptr);
        }
        rate.sleep();
    }
}
/**
 * 处理相机识别物体坐标
 */
void A2_Behavior_client::camera_pose_rec_callback(
    const geometry_msgs::msg::Point::SharedPtr point) {

    if (this->Receive_trig) {
        pose_receive_T = rclcpp::Time(now());
        this->receive_camera_point.x = point->x;
        this->receive_camera_point.y = point->y;
        this->receive_camera_point.z = point->z;
        // 当接收到transformed_pose时，模拟触发 'start' 动作
        auto start_msg = std_msgs::msg::String();
        start_msg.data = "start";  // 模拟发布 "start"
        robot_action_callback(std::make_shared<std_msgs::msg::String>(start_msg));
    }
}

// 任务执行
void A2_Behavior_client::robot_action_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "start") {
        Pub_pose_trig = true;
        Receive_trig = false;
        arm_action_fail_ = false;
        arm_action_start_ = false;
        arm_action_done_ = false;
        action_trig = 0;
        Time_check_trig_ = false;
    } else if (msg->data == "stop") {
        Pub_pose_trig = false;
        Receive_trig = true;
        action_trig = 100;
    }
    // 抓流程
}

// TODO 状态机程序

// 工具函数：根据输入pose和yaw偏移量，返回调整后的四元数
geometry_msgs::msg::Quaternion
adjust_pose_yaw(const geometry_msgs::msg::Pose &pose, double yaw_offset_rad) {
    double roll, pitch, yaw;
    tf2::Quaternion q_orig(pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q_orig);
    m.getRPY(roll, pitch, yaw);

    yaw += yaw_offset_rad;
    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, yaw);
    q_new.normalize();

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_new.x();
    q_msg.y = q_new.y();
    q_msg.z = q_new.z();
    q_msg.w = q_new.w();
    return q_msg;
}

void A2_Behavior_client::timer_callback() {
    if (rclcpp::ok() && (Pub_pose_trig)) {
        // 检查动作是否失败或等待超时，若是则重置流程
        if (arm_action_fail_) {
            RCLCPP_WARN(this->get_logger(), "动作失败，重置流程");
            Pub_pose_trig = false;
            action_trig = 0;
            Receive_trig = true;
        }
        if (((this->now() - pose_receive_T).seconds() > 1.0) &&
            !Time_check_trig_) {
            std::cout << "Time: " << (this->now() - pose_receive_T).seconds()
                      << std::endl;
            // std::cout<<"trig: "<
            Pub_pose_trig = false;
            Time_check_trig_ = true;
            Receive_trig = true;
            RCLCPP_WARN(this->get_logger(), "超时，重置流程");
        } else {
            Time_check_trig_ = true;
            RCLCPP_INFO(this->get_logger(), "接收到相机坐标");
            RCLCPP_INFO(this->get_logger(), "x: %f",
                        this->receive_camera_point.x);
            RCLCPP_INFO(this->get_logger(), "y: %f",
                        this->receive_camera_point.y);
            RCLCPP_INFO(this->get_logger(), "z: %f",
                        this->receive_camera_point.z);
        }
        static bool left_arm_action_trig = false;

        std_msgs::msg::String stop_msg;
        std_msgs::msg::String give_msg;                
        switch (action_trig) {
        case 0: {
            // 首先判断是左臂还是右臂
            if (receive_camera_point.x < 0.03) {
                left_arm_action_trig = true;
                std::cout << "left arm " << std::endl;
            } else {
                left_arm_action_trig = false;
                std::cout << "right arm " << std::endl;
            }

            action_trig++;
            break;
        }
        case 1: {
            // 第一步动作，到达瓶子上方
            RCLCPP_INFO(this->get_logger(), "Step 1: Move above the bottle");
            double yaw_offset = left_arm_action_trig ? -M_PI / 4.0 : M_PI / 4.0;
            std::cout << "yaw offset: " << yaw_offset * 180 / M_PI << std::endl;
            geometry_msgs::msg::Quaternion above_quat =
                adjust_pose_yaw(detect_pose, yaw_offset);

            std::vector<double> above_pose = {detect_pose.position.x,
                                              detect_pose.position.y,
                                              detect_pose.position.z,
                                              above_quat.x,
                                              above_quat.y,
                                              above_quat.z,
                                              above_quat.w};
            std::string arm_name =
                left_arm_action_trig ? "left_arm" : "right_arm";
            if (!arm_action_start_) {
                if (left_arm_action_trig) {
                    //left
                    std::vector<double> offset_pose = {-0.05, 0.02, 0.0,
                                                       0.0, 0.0, 0.0};
                    set_arm_target_pose(above_pose, offset_pose, arm_name);
                } else {
                    //right
                    std::vector<double> offset_pose = {-0.05, -0.02, 0.0,
                                                       0.0, 0.0, 0.0};
                    set_arm_target_pose(above_pose, offset_pose, arm_name);
                }
                arm_action_start_ = true;
            }
            if (arm_action_done_) {
                arm_action_start_ = false;
                action_trig++;
            }
            break;
        }
        case 2: {
            // 第二步动作，到达瓶子中间
            RCLCPP_INFO(this->get_logger(),
                        "Step 2: Move to the middle of the bottle");

            double yaw_offset = left_arm_action_trig ? -M_PI / 4.0 : M_PI / 4.0;
            std::cout << "yaw offset: " << yaw_offset * 180 / M_PI << std::endl;
            geometry_msgs::msg::Quaternion above_quat =
                adjust_pose_yaw(detect_pose, yaw_offset);

            std::vector<double> above_pose = {detect_pose.position.x + 0.01,
                                              detect_pose.position.y,
                                              detect_pose.position.z - 0.03,
                                              above_quat.x,
                                              above_quat.y,
                                              above_quat.z,
                                              above_quat.w};
            std::string arm_name =
                left_arm_action_trig ? "left_arm" : "right_arm";
            if (!arm_action_start_) {
                if (left_arm_action_trig) {
                    std::vector<double> offset_pose = {0.0, 0.0, 0.0,
                                                       0.0, 0.0, 0.0};
                    set_arm_target_pose(above_pose, offset_pose, arm_name);
                } else {
                    std::vector<double> offset_pose = {0.0, 0.0, 0.0,
                                                       0.0, 0.0, 0.0};
                    set_arm_target_pose(above_pose, offset_pose, arm_name);
                }
                arm_action_start_ = true;
            }

            if (arm_action_done_) {
                arm_action_start_ = false;
                action_trig++;
            }
            break;
        }
        case 3:
            // 第三步动作，抓取瓶子
            RCLCPP_INFO(this->get_logger(), "Step 3: Grab the bottle");
            if (left_arm_action_trig) {
                close_handle("left");
            } else {
                close_handle("right");
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
            // 发布抓住瓶子消息
            stop_msg = std_msgs::msg::String();
            stop_msg.data = "{\"message\": 1}";
            stop_publisher_->publish(stop_msg);
            action_trig++;
            break;
        case 4: {
            // 第四步动作，抬起瓶子到前方
            RCLCPP_INFO(this->get_logger(),
                        "Step 4: Lift the bottle to the front");
            std::string arm_name =
                left_arm_action_trig ? "left_arm" : "right_arm";
            if (!arm_action_start_) {
                set_arm_action("forward", arm_name);
                arm_action_start_ = true;
            }
            if (arm_action_done_) {
                action_trig++;
                arm_action_start_ = false;
            }
            break;
        }
        case 5:
            // 第五步动作，松开瓶子
            RCLCPP_INFO(this->get_logger(), "Step 5: Release the bottle");
            if (left_arm_action_trig) {
                open_handle("left");
            } else {
                open_handle("right");
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
            // 发布松开瓶子消息
            give_msg = std_msgs::msg::String();
            give_msg.data = "{\"message\": 1}";
            hand_give_publisher_->publish(give_msg);
            action_trig++;
            break;
        case 6: {
            // 第六步动作，手臂回到待执行位置
            RCLCPP_INFO(this->get_logger(),
                        "Step 6: Move arm to standby position");
            std::string arm_name =
                left_arm_action_trig ? "left_arm" : "right_arm";
            set_arm_action("ready", arm_name);
            ; // 假设有standby命名动作
            // action_trig = 0;                     //
            // 状态机重置，可根据需求修改
            action_trig = 100;
            Pub_pose_trig = false; // 停止发布
            Receive_trig = true;   // 接受坐标
            break;
        }
        default:
            action_trig = 100;
            break;
        }
    } else {
        // RCLCPP_INFO(this->get_logger(), "没有接收到相机坐标");
    }
}

// client 调用函数
void A2_Behavior_client::set_arm_target_pose(
    const std::vector<double> &target_pose,
    const std::vector<double> &offset_pose, const std::string &arm_name) {
    if (!arm_target_pose_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "set_arm_target_pose 服务不可用");
        return;
    }
    auto request =
        std::make_shared<custom_interface::srv::SetArmTargetPose::Request>();
    request->arm_name = arm_name;
    // target_pose: [x, y, z, qx, qy, qz, qw]
    if (target_pose.size() != 7) {
        RCLCPP_ERROR(this->get_logger(), "目标位姿参数数量错误，应为7");
        return;
    }
    if (offset_pose.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "偏移量参数数量错误，应为6");
        return;
    }
    request->arm_target_pose.position.x = target_pose[0];
    request->arm_target_pose.position.y = target_pose[1];
    request->arm_target_pose.position.z = target_pose[2];
    request->arm_target_pose.orientation.x = target_pose[3];
    request->arm_target_pose.orientation.y = target_pose[4];
    request->arm_target_pose.orientation.z = target_pose[5];
    request->arm_target_pose.orientation.w = target_pose[6];
    // 偏移量默认0
    std::copy_n(offset_pose.begin(),
                std::min(offset_pose.size(), request->offset_param.size()),
                request->offset_param.begin());

    arm_action_done_ = false;
    auto future = arm_target_pose_client_->async_send_request(
        request,
        [this](rclcpp::Client<custom_interface::srv::SetArmTargetPose>::
                   SharedFuture result_future) {
            auto response = result_future.get();
            if (response->arm_status) {
                arm_action_fail_ = false;
                RCLCPP_INFO(this->get_logger(), "目标位姿服务调用成功: %s",
                            response->message.c_str());
            } else {
                arm_action_fail_ = true;
                RCLCPP_WARN(this->get_logger(), "目标位姿服务调用失败: %s",
                            response->message.c_str());
            }
            arm_action_done_ = true;
        });
}

void A2_Behavior_client::set_arm_action(const std::string &action_name,
                                        const std::string &arm_name) {
    if (!arm_action_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "set_arm_action 服务不可用");
        return;
    }
    auto request =
        std::make_shared<custom_interface::srv::SetArmAction::Request>();
    request->arm_name = arm_name;
    request->arm_action = action_name;

    arm_action_done_ = false;
    auto future = arm_action_client_->async_send_request(
        request,
        [this](rclcpp::Client<custom_interface::srv::SetArmAction>::SharedFuture
                   result_future) {
            auto response = result_future.get();
            if (response->arm_status) {
                RCLCPP_INFO(this->get_logger(), "动作服务调用成功: %s",
                            response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "动作服务调用失败: %s",
                            response->message.c_str());
            }
            arm_action_done_ = true;
        });
}

void A2_Behavior_client::set_arm_joint_values(
    const std::vector<double> &joint_values, const std::string &arm_name) {
    if (!arm_joint_values_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "set_arm_joint_values 服务不可用");
        return;
    }
    auto request =
        std::make_shared<custom_interface::srv::SetArmJointValues::Request>();
    request->arm_name = arm_name;
    // std::array does not have clear(), so we assign values directly
    std::copy_n(joint_values.begin(),
                std::min(joint_values.size(), request->arm_joint_values.size()),
                request->arm_joint_values.begin());

    auto future = arm_joint_values_client_->async_send_request(
        request,
        [this](rclcpp::Client<custom_interface::srv::SetArmJointValues>::
                   SharedFuture result_future) {
            auto response = result_future.get();
            if (response->arm_status) {
                RCLCPP_INFO(this->get_logger(), "关节值服务调用成功: %s",
                            response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "关节值服务调用失败: %s",
                            response->message.c_str());
            }
        });
}

void A2_Behavior_client::close_handle(const std::string hand_name) {
    std_msgs::msg::String msg;
    std::cout << "close: " << hand_name << std::endl;
    if (hand_name == "left") {
        msg.data = "left_close";
    } else if (hand_name == "right") {
        msg.data = "right_close";
    }
    hand_control_publisher_->publish(msg);
}

void A2_Behavior_client::open_handle(const std::string hand_name) {
    std::cout << "open: " << hand_name << std::endl;
    std_msgs::msg::String msg;
    if (hand_name == "left") {
        msg.data = "left_open";
    } else if (hand_name == "right") {
        msg.data = "right_open";
    }
    hand_control_publisher_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<A2_Behavior_client>("a2_behaviors_client_node");

    RCLCPP_INFO(node->get_logger(), "开始发布client");
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "stop client");
    rclcpp::shutdown();
    return 0;
}