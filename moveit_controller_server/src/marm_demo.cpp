#include "marm_demo.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <thread>

// 构造函数
A2_Behavior_client::A2_Behavior_client(const std::string &name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "A2_Behavior_client node is started");
    init();
    
    // 加载运动学参数
    load_kinematics_parameters();
    
    // 等待参数设置完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 初始化MoveIt接口
    try {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "marm_group");
        RCLCPP_INFO(this->get_logger(), "MoveIt interface initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt interface: %s", e.what());
    }
    
    // 初始化TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // 创建一个周期定时器，绑定回调函数
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&A2_Behavior_client::timer_callback, this));
    // client初始化
    arm_target_pose_client_ =
        this->create_client<custom_interface::srv::SetArmTargetPose>(
            "set_arm_target_pose");
    arm_action_client_ =
        this->create_client<custom_interface::srv::SetArmAction>(
            "set_arm_action");
    
    // 初始化夹爪控制发布器
    gripper_publisher_ = this->create_publisher<std_msgs::msg::Bool>("gripper_control", 10);

    // 订阅 object_pose 话题（来自 rgb_detect_pose_node）
    object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "object_pose", 10,
        std::bind(&A2_Behavior_client::object_pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to object_pose topic");
}

void A2_Behavior_client::timer_callback() {
    // 检查动作是否失败，若是则重置流程
    if (arm_action_fail_) {
        RCLCPP_WARN(this->get_logger(), "动作失败，重置流程");
        action_trig = 0;
        arm_action_fail_ = false;
        arm_action_done_ = false;
        arm_action_start_ = false;
        return;
    }

    switch (action_trig) {
    case 0: {
        // 第一步：机械臂到达stand位置
        RCLCPP_INFO(this->get_logger(), "Step 0: Move to stand position");
        if (!arm_action_start_) {
            set_arm_action("stand", "marm_group");
            // 打开夹爪
            control_gripper(true); // false表示打开夹爪
            arm_action_start_ = true;
        }
        if (arm_action_done_) {
            arm_action_done_ = false;
            arm_action_start_ = false;
            action_trig++;
        }
        break;
    }
    case 1: {
        // 第二步：通过 object_pose 话题获取物体在 base_link 下的位置
        RCLCPP_INFO(this->get_logger(), "Step 1: Get object pose and calculate optimal pose");
        
        // 首次尝试时记录开始时间
        if (planning_retry_count_ == 0) {
            planning_start_time_ = std::chrono::steady_clock::now();
        }
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 从 object_pose 话题获取物体位置
        geometry_msgs::msg::PoseStamped cached_pose;
        bool pose_available = false;
        {
            std::lock_guard<std::mutex> lock(object_pose_mutex_);
            if (has_object_pose_) {
                cached_pose = latest_object_pose_;
                pose_available = true;
            }
        }

        if (!pose_available) {
            RCLCPP_WARN(this->get_logger(), "No object_pose received yet, waiting...");
            planning_retry_count_++;
            break;
        }

        // 检查 object_pose 时间戳是否新鲜（2秒内）
        auto pose_time = cached_pose.header.stamp;
        auto current_time = this->now();
        auto time_diff = (current_time - pose_time).seconds();
        if (time_diff > 2.0) {
            RCLCPP_WARN(this->get_logger(), "object_pose is stale (%.1f s old), waiting for fresh data...", time_diff);
            planning_retry_count_++;
            break;
        }

        // 记录当前抓取目标的颜色类型 frame_id (1=r, 2=g, 3=b)
        grasped_frame_id_ = cached_pose.header.frame_id;
        RCLCPP_INFO(this->get_logger(), "Target object frame_id: %s (1=Red, 2=Green, 3=Blue)", grasped_frame_id_.c_str());

        // object_pose 的数据已经是 base_link 坐标系下的位姿，直接使用
        double obj_x = cached_pose.pose.position.x + grasp_offset_x_ + current_grasp_x_offset_;
        double obj_y = cached_pose.pose.position.y + grasp_offset_y_;
        double obj_z = cached_pose.pose.position.z + grasp_offset_z_ + current_grasp_z_offset_;


        std::cout<<"offset_x: "<<grasp_offset_x_<<"  current_grasp_x_offset_: "<<current_grasp_x_offset_<<std::endl;
        std::cout<<"offset_z: "<<grasp_offset_z_<<"  current_grasp_z_offset_: "<<current_grasp_z_offset_<<std::endl;
        // 计算yaw角 - 物体与机械臂底座的夹角
        double yaw = atan2(obj_y, obj_x);
        double roll = 0.0;

        RCLCPP_INFO(this->get_logger(), "Object position: x=%.2f y=%.2f z=%.2f (frame_id=%s)",
                    obj_x, obj_y, obj_z, grasped_frame_id_.c_str());

        // 使用MoveIt选择合适的pitch角度并执行运动
        if (find_optimal_pitch(obj_x, obj_y, obj_z, roll, yaw, target_pose)) {
            RCLCPP_INFO(this->get_logger(), "Found optimal pose and executed arm movement successfully");
            planning_retry_count_ = 0;
            action_trig++; // 继续到步骤2（夹爪抓取）
        } else {
            planning_retry_count_++;
            RCLCPP_WARN(this->get_logger(), "Failed to find valid pitch angle or execute movement, retry");
        }
        break;
    }
    case 2: {
        // 夹爪抓取步骤
        RCLCPP_INFO(this->get_logger(), "Step 2: Close gripper to grasp object");
        // 关闭夹爪
        
        
        // 等待夹爪动作完成
        std::this_thread::sleep_for(std::chrono::milliseconds(800)); // 等待1秒
        control_gripper(false); // true表示关闭夹爪
        action_trig++;
        break;
    }
    case 3: {
        // 第三步：机械臂移动到stand位置（抓取物体后的安全位置）
        RCLCPP_INFO(this->get_logger(), "Step 3: Move to stand position with object");
        if (!arm_action_start_) {
            set_arm_action("stand", "marm_group");
            arm_action_start_ = true;
        }
        if (arm_action_done_) {
            arm_action_done_ = false;
            arm_action_start_ = false;
            action_trig++;  // 进入验证步骤
        }
        break;
    }
    case 4: {
        // 第四步：验证抓取是否成功（使用 frame_id 过滤，支持多物体场景）
        RCLCPP_INFO(this->get_logger(), "Step 4: Verify grasp success (target frame_id=%s)", grasped_frame_id_.c_str());
        
        if (grasp_retry_enabled_ && is_object_still_detected(grasp_detection_timeout_, grasped_frame_id_)) {
            // 仍然能检测到物体，说明抓取失败
            RCLCPP_WARN(this->get_logger(), "Object still detected! Grasp may have failed.");
            grasp_retry_count_++;
            
            if (grasp_retry_count_ <= grasp_max_retries_) {
                // 调整X位置偏移，增加一个步进量
                // current_grasp_x_offset_ += 0.01;
                current_grasp_z_offset_ += -0.015;
                RCLCPP_INFO(this->get_logger(), 
                    "Retry %d/%d: Adjusting X offset to %.3f m (step: %.3f m)", 
                    grasp_retry_count_, grasp_max_retries_, 
                    current_grasp_x_offset_, grasp_x_adjust_step_);
                
                // 打开夹爪
                control_gripper(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // 回到步骤1重新抓取
                action_trig = 1;
                RCLCPP_INFO(this->get_logger(), "Returning to Step 1 for re-grasp attempt");
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Max retries (%d) exceeded. Grasp failed. Proceeding anyway.", 
                    grasp_max_retries_);
                // 重置重试计数和偏移，继续后续流程
                grasp_retry_count_ = 0;
                current_grasp_x_offset_ = 0.0;
                action_trig++;
            }
        } else {
            // 物体未检测到，抓取成功
            RCLCPP_INFO(this->get_logger(), "Object not detected - Grasp successful!");
            // 重置重试计数和偏移
            grasp_retry_count_ = 0;
            current_grasp_x_offset_ = 0.0;
            action_trig++;
        }
        break;
    }
    case 5: {
        // 第五步：机械臂移动到put位置
        RCLCPP_INFO(this->get_logger(), "Step 5: Move to put position");
        if (!arm_action_start_) {
            set_arm_action("put", "marm_group");
            arm_action_start_ = true;
        }
        if (arm_action_done_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 等待1秒
            control_gripper(true); // true表示打开夹爪
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 等待1秒
            arm_action_done_ = false;
            arm_action_start_ = false;
            action_trig++;
        }
        break;
    }
    case 6: {
        // 第六步：机械臂回到home位置
        RCLCPP_INFO(this->get_logger(), "Step 6: Move to home position");
        if (!arm_action_start_) {
            set_arm_action("home", "marm_group");
            arm_action_start_ = true;
        }
        if (arm_action_done_) {
            arm_action_done_ = false;
            arm_action_start_ = false;
            RCLCPP_INFO(this->get_logger(), "流程完成，程序即将退出");
            // 退出程序
            action_trig = 0;
            //rclcpp::shutdown();
        }
        break;
    }
    default:
        break;
    }
}

void A2_Behavior_client::set_arm_action(const std::string &action_name,
                                        const std::string &arm_name) {
    if (!arm_action_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "set_arm_action 服务不可用");
        arm_action_fail_ = true;
        return;
    }
    
    auto request = std::make_shared<custom_interface::srv::SetArmAction::Request>();
    request->arm_action = action_name;
    request->arm_name = arm_name;

    arm_action_done_ = false;
    auto future = arm_action_client_->async_send_request(
        request,
        [this, action_name](rclcpp::Client<custom_interface::srv::SetArmAction>::SharedFuture result_future) {
            try {
                auto response = result_future.get();
                if (response->arm_status) {
                    RCLCPP_INFO(this->get_logger(), "Action '%s' executed successfully", action_name.c_str());
                    arm_action_done_ = true;
                    arm_action_fail_ = false;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Action '%s' failed: %s", 
                                action_name.c_str(), response->message.c_str());
                    arm_action_fail_ = true;
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call exception: %s", e.what());
                arm_action_fail_ = true;
            }
        });
}

void A2_Behavior_client::set_arm_target_pose(const std::vector<double> &target_pose,
                                             const std::vector<double> &offset_pose,
                                             const std::string &arm_name) {
    if (!arm_target_pose_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "set_arm_target_pose 服务不可用");
        arm_action_fail_ = true;
        return;
    }
    
    auto request = std::make_shared<custom_interface::srv::SetArmTargetPose::Request>();
    request->arm_name = arm_name;
    request->use_cartesian = false;
    
    // 设置位姿 [x, y, z, qx, qy, qz, qw]
    if (target_pose.size() >= 7) {
        request->arm_target_pose.position.x = target_pose[0];
        request->arm_target_pose.position.y = target_pose[1];
        request->arm_target_pose.position.z = target_pose[2];
        request->arm_target_pose.orientation.x = target_pose[3];
        request->arm_target_pose.orientation.y = target_pose[4];
        request->arm_target_pose.orientation.z = target_pose[5];
        request->arm_target_pose.orientation.w = target_pose[6];
    }
    
    // 设置偏移参数
    if (offset_pose.size() >= 6) {
        for (size_t i = 0; i < 6 && i < offset_pose.size(); ++i) {
            request->offset_param[i] = offset_pose[i];
        }
    }

    arm_action_done_ = false;
    auto future = arm_target_pose_client_->async_send_request(
        request,
        [this](rclcpp::Client<custom_interface::srv::SetArmTargetPose>::SharedFuture result_future) {
            try {
                auto response = result_future.get();
                if (response->arm_status) {
                    RCLCPP_INFO(this->get_logger(), "Target pose set successfully");
                    arm_action_done_ = true;
                    arm_action_fail_ = false;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Set target pose failed: %s", response->message.c_str());
                    arm_action_fail_ = true;
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call exception: %s", e.what());
                arm_action_fail_ = true;
            }
        });
}

// 新增：使用MoveIt迭代选择合适的pitch角度
bool A2_Behavior_client::find_optimal_pitch(double x, double y, double z, double roll, double yaw, 
                                           geometry_msgs::msg::Pose &optimal_pose) {
    if (!move_group_interface_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized");
        return false;
    }
    
    // 设置位置
    optimal_pose.position.x = x;
    optimal_pose.position.y = y;
    optimal_pose.position.z = z;
    
    // 简化的pitch角度候选列表：从10度到50度，步长1度
    std::vector<double> pitch_candidates;
    for (int angle = 50; angle >= 10; angle -= 1) {
        pitch_candidates.push_back(angle * M_PI / 180.0);
    }
    
    RCLCPP_INFO(this->get_logger(), "Testing %zu pitch angles from 10° to 50°...", pitch_candidates.size());
    
    for (double pitch : pitch_candidates) {
        RCLCPP_INFO(this->get_logger(), "Testing pitch: %.1f°", pitch * 180.0 / M_PI);
        
        // 创建四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        optimal_pose.orientation.x = q.x();
        optimal_pose.orientation.y = q.y();
        optimal_pose.orientation.z = q.z();
        optimal_pose.orientation.w = q.w();
        
        // 配置MoveIt规划参数（从YAML文件加载）
        move_group_interface_->setPlanningTime(planning_time_);
        move_group_interface_->setPoseTarget(optimal_pose);
        move_group_interface_->setGoalPositionTolerance(goal_position_tolerance_);
        move_group_interface_->setGoalOrientationTolerance(goal_orientation_tolerance_);
        move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
        move_group_interface_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
        move_group_interface_->setNumPlanningAttempts(planning_attempts_);
        
        // 尝试规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto planning_result = move_group_interface_->plan(plan);
        
        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), " Found valid solution with pitch=%.1f°", pitch * 180.0 / M_PI);
            
            // 发布目标位姿的tf，方便查看规划结果
            publish_target_pose_tf(optimal_pose, "planned_target_pose");
            RCLCPP_INFO(this->get_logger(), "Published planned target pose tf: 'planned_target_pose'");
            
            // 执行机械臂运动
            auto execute_result = move_group_interface_->execute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), " Arm execution completed successfully!");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), " Arm execution failed!");
                return false;
            }
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), " No valid pitch angle found for position (%.3f, %.3f, %.3f)", x, y, z);
    return false;
}

void A2_Behavior_client::publish_target_pose_tf(const geometry_msgs::msg::Pose &pose, const std::string &frame_id) {
    if (!tf_broadcaster_) {
        return;
    }
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "base_footprint";
    transform_stamped.child_frame_id = frame_id;
    
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation = pose.orientation;
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

// 加载运动学参数的函数
void A2_Behavior_client::load_kinematics_parameters() {
    RCLCPP_INFO(this->get_logger(), "Loading kinematics parameters from YAML...");
    
    try {
        // 配置文件路径（使用 ament_index_cpp 动态获取包路径，避免硬编码）
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("moveit_controller_server");
        std::string config_file_path = pkg_share_dir + "/config/load_kinematics.yaml";
        
        // 检查文件是否存在
        std::ifstream file_check(config_file_path);
        if (!file_check.good()) {
            RCLCPP_WARN(this->get_logger(), "Config file not found, using default parameters");
            load_default_kinematics_parameters();
            return;
        }
        file_check.close();
        
        // 读取YAML配置文件
        YAML::Node config = YAML::LoadFile(config_file_path);
        auto marm_group_config = config["robot_description_kinematics"]["marm_group"];
        std::string base_param = "robot_description_kinematics.marm_group";
        
        // 设置运动学参数
        if (marm_group_config["kinematics_solver"]) {
            this->declare_parameter(base_param + ".kinematics_solver", 
                                  marm_group_config["kinematics_solver"].as<std::string>());
        }
        if (marm_group_config["kinematics_solver_search_resolution"]) {
            this->declare_parameter(base_param + ".kinematics_solver_search_resolution", 
                                  marm_group_config["kinematics_solver_search_resolution"].as<double>());
        }
        if (marm_group_config["kinematics_solver_timeout"]) {
            this->declare_parameter(base_param + ".kinematics_solver_timeout", 
                                  marm_group_config["kinematics_solver_timeout"].as<double>());
        }
        if (marm_group_config["kinematics_solver_attempts"]) {
            this->declare_parameter(base_param + ".kinematics_solver_attempts", 
                                  marm_group_config["kinematics_solver_attempts"].as<int>());
        }
        if (marm_group_config["position_only_ik"]) {
            this->declare_parameter(base_param + ".position_only_ik", 
                                  marm_group_config["position_only_ik"].as<bool>());
        }
        
        // 加载MoveIt规划参数
        auto moveit_config = config["moveit_planning"];
        if (moveit_config) {
            velocity_scaling_factor_ = moveit_config["velocity_scaling_factor"] ? 
                                     moveit_config["velocity_scaling_factor"].as<double>() : 0.4;
            acceleration_scaling_factor_ = moveit_config["acceleration_scaling_factor"] ? 
                                         moveit_config["acceleration_scaling_factor"].as<double>() : 0.2;
            planning_time_ = moveit_config["planning_time"] ? 
                           moveit_config["planning_time"].as<double>() : 0.2;
            planning_attempts_ = moveit_config["planning_attempts"] ? 
                               moveit_config["planning_attempts"].as<int>() : 50;
            goal_position_tolerance_ = moveit_config["goal_position_tolerance"] ? 
                                     moveit_config["goal_position_tolerance"].as<double>() : 0.01;
            goal_orientation_tolerance_ = moveit_config["goal_orientation_tolerance"] ? 
                                        moveit_config["goal_orientation_tolerance"].as<double>() : 0.1;
        } else {
            // 使用默认的MoveIt参数
            velocity_scaling_factor_ = 0.4;
            acceleration_scaling_factor_ = 0.2;
            planning_time_ = 0.2;
            planning_attempts_ = 50;
            goal_position_tolerance_ = 0.01;
            goal_orientation_tolerance_ = 0.1;
        }
        
        // 加载抓取位置微调参数
        auto grasp_offset_config = config["grasp_offset"];
        if (grasp_offset_config) {
            grasp_offset_x_ = grasp_offset_config["x"] ? 
                            grasp_offset_config["x"].as<double>() : 0.02;
            grasp_offset_y_ = grasp_offset_config["y"] ? 
                            grasp_offset_config["y"].as<double>() : 0.0;
            grasp_offset_z_ = grasp_offset_config["z"] ? 
                            grasp_offset_config["z"].as<double>() : 0.01;
        } else {
            grasp_offset_x_ = 0.02;
            grasp_offset_y_ = 0.0;
            grasp_offset_z_ = 0.01;
        }
        
        // 加载抓取重试参数
        auto grasp_retry_config = config["grasp_retry"];
        if (grasp_retry_config) {
            grasp_retry_enabled_ = grasp_retry_config["enabled"] ? 
                                 grasp_retry_config["enabled"].as<bool>() : true;
            grasp_max_retries_ = grasp_retry_config["max_retries"] ? 
                               grasp_retry_config["max_retries"].as<int>() : 3;
            grasp_x_adjust_step_ = grasp_retry_config["x_adjust_step"] ? 
                                 grasp_retry_config["x_adjust_step"].as<double>() : 0.01;
            grasp_detection_timeout_ = grasp_retry_config["detection_timeout"] ? 
                                     grasp_retry_config["detection_timeout"].as<double>() : 2.0;
        } else {
            grasp_retry_enabled_ = true;
            grasp_max_retries_ = 3;
            grasp_x_adjust_step_ = 0.01;
            grasp_detection_timeout_ = 2.0;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded kinematics parameters from YAML");
        RCLCPP_INFO(this->get_logger(), "MoveIt Planning Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Velocity scaling: %.2f", velocity_scaling_factor_);
        RCLCPP_INFO(this->get_logger(), "  - Acceleration scaling: %.2f", acceleration_scaling_factor_);
        RCLCPP_INFO(this->get_logger(), "  - Planning time: %.2f s", planning_time_);
        RCLCPP_INFO(this->get_logger(), "  - Planning attempts: %d", planning_attempts_);
        RCLCPP_INFO(this->get_logger(), "Grasp Offset Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - X offset: %.3f m", grasp_offset_x_);
        RCLCPP_INFO(this->get_logger(), "  - Y offset: %.3f m", grasp_offset_y_);
        RCLCPP_INFO(this->get_logger(), "  - Z offset: %.3f m", grasp_offset_z_);
        RCLCPP_INFO(this->get_logger(), "Grasp Retry Parameters:");
        RCLCPP_INFO(this->get_logger(), "  - Retry enabled: %s", grasp_retry_enabled_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  - Max retries: %d", grasp_max_retries_);
        RCLCPP_INFO(this->get_logger(), "  - X adjust step: %.3f m", grasp_x_adjust_step_);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load YAML config: %s", e.what());
        load_default_kinematics_parameters();
    }
}

void A2_Behavior_client::load_default_kinematics_parameters() {
    std::string base_param = "robot_description_kinematics.marm_group";
    
    this->declare_parameter(base_param + ".kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    this->declare_parameter(base_param + ".kinematics_solver_search_resolution", 0.001);
    this->declare_parameter(base_param + ".kinematics_solver_timeout", 0.01);
    this->declare_parameter(base_param + ".kinematics_solver_attempts", 3);
    this->declare_parameter(base_param + ".position_only_ik", false);
    
    // 设置默认的MoveIt规划参数
    velocity_scaling_factor_ = 0.4;
    acceleration_scaling_factor_ = 0.2;
    planning_time_ = 0.2;
    planning_attempts_ = 50;
    goal_position_tolerance_ = 0.01;
    goal_orientation_tolerance_ = 0.1;
    
    // 设置默认的抓取位置微调参数
    grasp_offset_x_ = 0.01;
    grasp_offset_y_ = 0.0;
    grasp_offset_z_ = -0.02;
    
    // 设置默认的抓取重试参数
    grasp_retry_enabled_ = true;
    grasp_max_retries_ = 3;
    grasp_x_adjust_step_ = 0.01;
    grasp_detection_timeout_ = 2.0;
    
    RCLCPP_INFO(this->get_logger(), "Default kinematics parameters loaded");
}

void A2_Behavior_client::control_gripper(bool close_gripper) {
    if (!gripper_publisher_) {
        RCLCPP_ERROR(this->get_logger(), "Gripper publisher not initialized");
        return;
    }
    
    auto msg = std_msgs::msg::Bool();
    msg.data = close_gripper;
    
    gripper_publisher_->publish(msg);
    
    if (close_gripper) {
        RCLCPP_INFO(this->get_logger(), "Gripper control: 打开夹爪 (Close gripper) - Set joint6 to -50 degrees");
    } else {
        RCLCPP_INFO(this->get_logger(), "Gripper control: 关闭夹爪 (Open gripper) - Set joint6 to 10 degrees");
    }
}

// object_pose 话题回调函数
void A2_Behavior_client::object_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(object_pose_mutex_);
    latest_object_pose_ = *msg;
    has_object_pose_ = true;
    // RCLCPP_INFO(this->get_logger(),"pose: x=%.3f, y=%.3f, z=%.3f",
    //             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

// 检测物体是否仍然存在（用于验证抓取是否成功）
// target_frame_id: 指定要检测的颜色类型 frame_id（1=r, 2=g, 3=b），
//                  空字符串表示不过滤，兼容单物体场景
bool A2_Behavior_client::is_object_still_detected(double timeout_seconds, const std::string& target_frame_id) {
    if (target_frame_id.empty()) {
        RCLCPP_INFO(this->get_logger(), "Checking if any object is still detected (timeout: %.1f s)...", timeout_seconds);
    } else {
        RCLCPP_INFO(this->get_logger(), "Checking if object (frame_id=%s) is still detected (timeout: %.1f s)...",
                    target_frame_id.c_str(), timeout_seconds);
    }
    
    auto start_time = std::chrono::steady_clock::now();
    int detection_attempts = 0;
    int successful_detections = 0;
    const int required_detections = 3;  // 需要连续检测到3次才认为物体存在
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count() / 1000.0;
        
        if (elapsed > timeout_seconds) {
            RCLCPP_INFO(this->get_logger(), "Detection timeout. Object not consistently detected.");
            return false;
        }
        
        // 使用 object_pose 话题进行检测（支持 frame_id 过滤）
        geometry_msgs::msg::PoseStamped cached_pose;
        bool pose_available = false;
        {
            std::lock_guard<std::mutex> lock(object_pose_mutex_);
            if (has_object_pose_) {
                cached_pose = latest_object_pose_;
                pose_available = true;
            }
        }

        if (pose_available) {
            // 检查时间戳是否新鲜（1秒内）
            auto pose_time = cached_pose.header.stamp;
            auto current_time = this->now();
            auto time_diff = (current_time - pose_time).seconds();

            if (time_diff < 1.0) {
                // 多物体场景：通过 frame_id 过滤，确保检测到的是同一个颜色类型的物体
                bool frame_id_match = target_frame_id.empty() || 
                                     (cached_pose.header.frame_id == target_frame_id);

                if (frame_id_match) {
                    successful_detections++;
                    RCLCPP_DEBUG(this->get_logger(), "Object detected (frame_id=%s, %d/%d)",
                               cached_pose.header.frame_id.c_str(),
                               successful_detections, required_detections);

                    if (successful_detections >= required_detections) {
                        RCLCPP_INFO(this->get_logger(),
                            "Object consistently detected (frame_id=%s): pos(%.3f, %.3f, %.3f)",
                            cached_pose.header.frame_id.c_str(),
                            cached_pose.pose.position.x,
                            cached_pose.pose.position.y,
                            cached_pose.pose.position.z);
                        return true;
                    }
                } else {
                    // frame_id 不匹配，说明检测到的是其他颜色的物体，不计入
                    RCLCPP_DEBUG(this->get_logger(),
                        "Object detected but frame_id mismatch: got %s, expected %s",
                        cached_pose.header.frame_id.c_str(), target_frame_id.c_str());
                    successful_detections = 0;
                }
            } else {
                // 数据太旧，不计入检测
                successful_detections = 0;
            }
        } else {
            // 没有收到 object_pose
            successful_detections = 0;
        }
        
        detection_attempts++;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    return false;
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<A2_Behavior_client>("marm_demo");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}