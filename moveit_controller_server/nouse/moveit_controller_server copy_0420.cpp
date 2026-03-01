#include "custom_interface/srv/set_arm_action.hpp"
#include "custom_interface/srv/set_arm_joint_values.hpp"
#include "custom_interface/srv/set_arm_target_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <mutex>                // 添加互斥锁头文件
#include <rclcpp/executors.hpp> // 添加多线程执行器头文件
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class FK_Control_Server : public rclcpp::Node {
  public:
    explicit FK_Control_Server(const std::string &name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "节点已启动: %s", name.c_str());

        // 创建服务
        set_arm_action_server_ =
            this->create_service<custom_interface::srv::SetArmAction>(
                "set_arm_action",
                std::bind(&FK_Control_Server::set_arm_action_callback, this,
                          std::placeholders::_1, std::placeholders::_2));

        set_arm_joint_values_server_ =
            this->create_service<custom_interface::srv::SetArmJointValues>(
                "set_arm_joint_values",
                std::bind(&FK_Control_Server::set_arm_joint_values_callback,
                          this, std::placeholders::_1, std::placeholders::_2));

        set_arm_target_pose_server_ =
            this->create_service<custom_interface::srv::SetArmTargetPose>(
                "set_arm_target_pose",
                std::bind(&FK_Control_Server::set_arm_target_pose_callback,
                          this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "所有服务已创建并启动");
    }

  private:
    // 声明服务
    rclcpp::Service<custom_interface::srv::SetArmAction>::SharedPtr
        set_arm_action_server_;
    rclcpp::Service<custom_interface::srv::SetArmJointValues>::SharedPtr
        set_arm_joint_values_server_;
    rclcpp::Service<custom_interface::srv::SetArmTargetPose>::SharedPtr
        set_arm_target_pose_server_;

    // 缓存MoveGroupInterface对象的映射
    std::map<std::string,
             std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
        move_group_cache_;

    // 四元数转欧拉角（角度），orientation_顺序为[x, y, z, w]
    static void Orientation2RPY(const std::vector<double> &orientation_,
                                double &roll, double &pitch, double &yaw) {
        if (orientation_.size() != 4) {
            roll = pitch = yaw = 0.0;
            return;
        }
        tf2::Quaternion q(orientation_[0], orientation_[1], orientation_[2],
                          orientation_[3]);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        // 转为角度
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;
    }

    // 欧拉角（弧度）转四元数，orientation_顺序为[x, y, z, w]
    static void RPY2Orientation(double roll, double pitch, double yaw,
                                std::vector<double> &orientation_) {
        tf2::Quaternion q;
        q.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
        orientation_.resize(4);
        orientation_[0] = q.x();
        orientation_[1] = q.y();
        orientation_[2] = q.z();
        orientation_[3] = q.w();
    }

    // 获取或创建MoveGroupInterface对象
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
    get_move_group(const std::string &arm_name) {
        if (move_group_cache_.find(arm_name) == move_group_cache_.end()) {
            try {
                move_group_cache_[arm_name] = std::make_shared<
                    moveit::planning_interface::MoveGroupInterface>(
                    this->shared_from_this(), arm_name);
                RCLCPP_INFO(this->get_logger(),
                            "已为机械臂 '%s' 创建MoveGroupInterface",
                            arm_name.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "创建MoveGroupInterface失败: %s", e.what());
                return nullptr;
            }
        }
        // 优化机械臂速度
        auto move_group = move_group_cache_[arm_name];
        move_group->setMaxVelocityScalingFactor(0.5); // 设置最大速度缩放因子
        move_group->setMaxAccelerationScalingFactor(
            0.5); // 设置最大加速度缩放因子
        return move_group;
    }

    // 通用规划和执行函数
    bool plan_and_execute(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
            move_group,
        std::string &message) {
        if (!move_group) {
            message = "无效的MoveGroupInterface对象";
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 设置最大规划时间以提高效率
        move_group->setPlanningTime(5.0);

        RCLCPP_INFO(this->get_logger(), "开始规划路径...");

        // 使用超时规划，提高稳定性
        auto start_time = std::chrono::system_clock::now();
        auto plan_result = move_group->plan(my_plan);
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        RCLCPP_DEBUG(this->get_logger(), "规划用时: %.3f秒",
                     elapsed_seconds.count());

        bool success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            // 检查规划结果中的路径点数量
            if (my_plan.trajectory_.joint_trajectory.points.empty()) {
                message = "规划生成了空轨迹";
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行...");
            auto execution_result = move_group->execute(my_plan);

            if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "执行成功");
                message = "规划和执行成功";
                return true;
            } else {
                message =
                    "执行失败，错误码: " + std::to_string(execution_result.val);
                RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
                return false;
            }
        } else {
            message = "规划失败，错误码: " + std::to_string(plan_result.val);
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            return false;
        }
    }

    // INFO 固定action控制
    void set_arm_action_callback(
        const std::shared_ptr<custom_interface::srv::SetArmAction::Request>
            request,
        std::shared_ptr<custom_interface::srv::SetArmAction::Response>
            response) {
        std::cout << "\033[1;32m-----------------------------------------------"
                     "------\033[0m"
                  << std::endl;
        RCLCPP_INFO(this->get_logger(), "收到set_arm_action服务请求");
        RCLCPP_INFO(this->get_logger(), "机械臂名称: %s, 动作名称: %s",
                    request->arm_name.c_str(), request->arm_action.c_str());

        if (request->arm_name.empty() || request->arm_action.empty()) {
            response->arm_status = false;
            response->message = "机械臂名称或动作名称为空";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 获取或创建MoveGroupInterface
        auto move_group = get_move_group(request->arm_name);
        if (!move_group) {
            response->arm_status = false;
            response->message = "创建机械臂控制接口失败";
            return;
        }

        try {
            // 根据请求的动作名称设置目标
            move_group->setNamedTarget(request->arm_action);

            // 规划和执行动作
            response->arm_status =
                plan_and_execute(move_group, response->message);
        } catch (const std::exception &e) {
            response->arm_status = false;
            response->message = std::string("动作控制异常: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // INFO 关节角度控制
    void set_arm_joint_values_callback(
        const std::shared_ptr<custom_interface::srv::SetArmJointValues::Request>
            request,
        std::shared_ptr<custom_interface::srv::SetArmJointValues::Response>
            response) {
        RCLCPP_INFO(this->get_logger(), "收到set_arm_joint_values服务请求");
        RCLCPP_INFO(this->get_logger(), "机械臂名称: %s",
                    request->arm_name.c_str());

        if (request->arm_name.empty()) {
            response->arm_status = false;
            response->message = "机械臂名称为空";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        const auto &joint_array = request->arm_joint_values;
        std::vector<double> joint_group_positions(joint_array.begin(),
                                                  joint_array.end());

        if (joint_group_positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "接收到空的关节值列表");
            response->arm_status = false;
            response->message = "接收到空的关节值列表";
            return;
        }

        // 日志输出关节值
        std::stringstream joint_values_ss;
        joint_values_ss << "关节值: [";
        for (size_t i = 0; i < joint_group_positions.size(); ++i) {
            joint_values_ss << joint_group_positions[i];
            if (i < joint_group_positions.size() - 1) {
                joint_values_ss << ", ";
            }
        }
        joint_values_ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", joint_values_ss.str().c_str());

        // 获取或创建MoveGroupInterface
        auto move_group = get_move_group(request->arm_name);
        if (!move_group) {
            response->arm_status = false;
            response->message = "创建机械臂控制接口失败";
            return;
        }

        // 确认关节数量是否匹配
        auto joint_names = move_group->getJointNames();
        if (joint_group_positions.size() != joint_names.size()) {
            response->arm_status = false;
            response->message = "关节数量不匹配：提供了 " +
                                std::to_string(joint_group_positions.size()) +
                                " 个关节值，但机械臂有 " +
                                std::to_string(joint_names.size()) + " 个关节";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        try {
            move_group->setJointValueTarget(joint_group_positions);
            response->arm_status =
                plan_and_execute(move_group, response->message);
        } catch (const std::exception &e) {
            response->arm_status = false;
            response->message = std::string("关节角度控制异常: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // INFO 位姿控制
    void set_arm_target_pose_callback(
        const std::shared_ptr<custom_interface::srv::SetArmTargetPose::Request>
            request,
        std::shared_ptr<custom_interface::srv::SetArmTargetPose::Response>
            response) {
        std::cout << "\033[1;32m-----------------------------------------------"
                     "------\033[0m"
                  << std::endl;
        RCLCPP_INFO(this->get_logger(), "收到set_arm_target_pose服务请求");
        RCLCPP_INFO(this->get_logger(), "机械臂名称: %s",
                    request->arm_name.c_str());

        if (request->arm_name.empty()) {
            response->arm_status = false;
            response->message = "机械臂名称为空";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 确认姿态的有效性 (四元数范数应接近1)
        const auto &q = request->arm_target_pose.orientation;
        double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (std::abs(norm - 1.0) > 0.01) {
            RCLCPP_WARN(this->get_logger(),
                        "四元数不是单位四元数，范数为: %.3f", norm);
        }

        // 使用ANSI转义序列打印彩色日志（蓝色）
        printf("\033[1;32m目标位置: [%.3f, %.3f, %.3f], 姿态: [%.3f, %.3f, "
               "%.3f, %.3f]\033[0m\n",
               request->arm_target_pose.position.x,
               request->arm_target_pose.position.y,
               request->arm_target_pose.position.z,
               request->arm_target_pose.orientation.x,
               request->arm_target_pose.orientation.y,
               request->arm_target_pose.orientation.z,
               request->arm_target_pose.orientation.w);

        RCLCPP_INFO(
            this->get_logger(),
            "position偏移量:[%.3f, %.3f,%.3f], rpy偏移量:[%.3f, %.3f, %.3f]",
            request->offset_param[0], request->offset_param[1],
            request->offset_param[2], request->offset_param[3],
            request->offset_param[4], request->offset_param[5]);

        // 获取或创建MoveGroupInterface
        auto move_group = get_move_group(request->arm_name);
        if (!move_group) {
            response->arm_status = false;
            response->message = "创建机械臂控制接口失败";
            return;
        }

        try {
            // 设置规划参考坐标系为 "base_arm_link"
            move_group->setPoseReferenceFrame("arm_base_link");

            // 打印参考坐标系
            RCLCPP_INFO(this->get_logger(), "当前规划参考坐标系: %s",
                        move_group->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "目标位姿参考坐标系: %s",
                        "base_arm_link");
            RCLCPP_INFO(this->get_logger(), "当前执行坐标： %s",
                        move_group->getEndEffectorLink().c_str());

            // ===== 新增：设置更严格的目标容差 =====
            move_group->setGoalPositionTolerance(0.001);    // 2mm
            move_group->setGoalOrientationTolerance(0.005); // 约0.3度

            geometry_msgs::msg::Pose offset_target_pose;
            geometry_msgs::msg::Pose target_pose =
                request->arm_target_pose; // 目标位姿
            // 将目标四元数转化为rpy
            double target_roll, target_pitch, target_yaw;
            Orientation2RPY(
                {target_pose.orientation.x, target_pose.orientation.y,
                 target_pose.orientation.z, target_pose.orientation.w},
                target_roll, target_pitch, target_yaw);
            // 进行rpy调整
            std::vector<double> offset_target_que;
            RPY2Orientation(target_roll + request->offset_param[3],
                            target_pitch + request->offset_param[4],
                            target_yaw + request->offset_param[5],
                            offset_target_que);

            // ====== 关键修改：将偏移量从目标点坐标系变换到全局坐标系 ======
            tf2::Quaternion q_target(
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);
            tf2::Matrix3x3 rot_matrix(q_target);

            // 构造局部偏移向量
            tf2::Vector3 local_offset(request->offset_param[0],
                                      request->offset_param[1],
                                      request->offset_param[2]);

            // 变换到全局坐标系
            tf2::Vector3 global_offset = rot_matrix * local_offset;

            // 新的目标位姿
            offset_target_pose.position.x =
                target_pose.position.x + global_offset.x();
            offset_target_pose.position.y =
                target_pose.position.y + global_offset.y();
            offset_target_pose.position.z =
                target_pose.position.z + global_offset.z();
            offset_target_pose.orientation.x = offset_target_que[0];
            offset_target_pose.orientation.y = offset_target_que[1];
            offset_target_pose.orientation.z = offset_target_que[2];
            offset_target_pose.orientation.w = offset_target_que[3];

            // 使用ANSI转义序列打印彩色日志（绿色）
            printf("\033[1;32moffset目标位置: [%.3f, %.3f, %.3f], offset姿态: "
                   "[%.3f, %.3f, %.3f, %.3f]\033[0m\n",
                   offset_target_pose.position.x, offset_target_pose.position.y,
                   offset_target_pose.position.z,
                   offset_target_pose.orientation.x,
                   offset_target_pose.orientation.y,
                   offset_target_pose.orientation.z,
                   offset_target_pose.orientation.w);

            if (request->use_cartesian) {
                std::cout << "\033[1;32m 使用笛卡尔坐标系： " << std::endl;
                // ======= 新增：使用笛卡尔路径规划 =======
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(offset_target_pose);

                moveit_msgs::msg::RobotTrajectory trajectory_msg;
                const double eef_step = 0.01;      // 1cm 步长
                const double jump_threshold = 0.0; // 不限制关节跳跃
                double fraction = move_group->computeCartesianPath(
                    waypoints, eef_step, jump_threshold, trajectory_msg);

                if (fraction < 0.99) {
                    response->arm_status = false;
                    response->message = "笛卡尔路径规划失败，规划完成率: " +
                                        std::to_string(fraction);
                    RCLCPP_ERROR(this->get_logger(), "%s",
                                 response->message.c_str());
                    return;
                }

                moveit::planning_interface::MoveGroupInterface::Plan
                    cartesian_plan;
                cartesian_plan.trajectory_ = trajectory_msg;

                RCLCPP_INFO(this->get_logger(),
                            "笛卡尔路径规划成功，开始执行...");
                auto execution_result = move_group->execute(cartesian_plan);

                if (execution_result ==
                    moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "执行成功");
                    response->arm_status = true;
                    response->message = "笛卡尔路径规划和执行成功";
                } else {
                    response->arm_status = false;
                    response->message = "笛卡尔路径执行失败，错误码: " +
                                        std::to_string(execution_result.val);
                    RCLCPP_ERROR(this->get_logger(), "%s",
                                 response->message.c_str());
                }
                // ======= 新增结束 =======
                std::cout << "\033[0m" << std::endl;
            } else {
                std::cout << "不使用笛卡尔坐标" << std::endl;
                response->arm_status =
                    plan_and_execute(move_group, response->message);
            }

        } catch (const std::exception &e) {
            response->arm_status = false;
            response->message = std::string("目标位姿控制异常: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 使用智能指针管理节点生命周期
    auto node =
        std::make_shared<FK_Control_Server>("moveit_controller_server_node");

    // 处理来自ROS2的回调
    RCLCPP_INFO(node->get_logger(), "开始接收服务请求...");
    rclcpp::spin(node);

    // 关闭ROS2系统
    RCLCPP_INFO(node->get_logger(), "关闭节点");
    rclcpp::shutdown();
    return 0;
}