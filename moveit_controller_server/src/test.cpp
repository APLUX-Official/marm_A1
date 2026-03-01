#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

class MoveItDemo : public rclcpp::Node
{
public:
    MoveItDemo() : Node("moveit_demo")
    {
        RCLCPP_INFO(this->get_logger(), "正在初始化MoveIt Demo...");

        // 延迟初始化MoveGroup，避免在构造函数中使用shared_from_this()
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                this->init_timer_->cancel();
                this->initializeMoveGroup();
            });
    }

private:
    void initializeMoveGroup()
    {
        // 创建MoveGroup接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "marm_group");

        // 设置规划参数
        move_group_->setPlanningTime(10);       // 增加规划时间
        move_group_->setNumPlanningAttempts(4); // 增加尝试次数
        move_group_->setGoalPositionTolerance(0.002);
        move_group_->setGoalOrientationTolerance(0.01);

        // 设置规划器（使用更稳定的规划器）
        move_group_->setPlannerId("TRRT"); // 更改为PRM，质量更好但稍慢

        // 确保规划坐标系正确
        move_group_->setPoseReferenceFrame("world");

        // 清除所有路径约束
        move_group_->clearPathConstraints();
        move_group_->clearPoseTargets();

        RCLCPP_INFO(this->get_logger(), "MoveIt Demo 初始化完成!");
        RCLCPP_INFO(this->get_logger(), "规划坐标系: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "末端执行器: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "规划器: %s", move_group_->getPlannerId().c_str());

        // 等待一段时间确保所有服务就绪
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 显示当前位姿
        // showCurrentPose();

        initialized_ = true;
    }

public:
    // 显示当前位姿
    void showCurrentPose()
    {
        if (!initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "MoveGroup 尚未初始化!");
            return;
        }

        try
        {
            // 尝试获取当前状态，如果失败则设置默认状态
            auto current_state = move_group_->getCurrentState(3.0); // 3秒超时
            if (!current_state)
            {
                RCLCPP_WARN(this->get_logger(), "无法获取当前状态，使用默认状态");
                // 设置默认关节状态
                std::vector<double> default_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0};
                current_state = move_group_->getCurrentState();
                current_state->setJointGroupPositions("marm_group", default_joint_values);
                move_group_->setJointValueTarget(default_joint_values);
            }

            auto current_pose = move_group_->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "当前位姿:");
            RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f, y=%.3f, z=%.3f",
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z);

            // 将四元数转换为RPY
            tf2::Quaternion quat;
            tf2::fromMsg(current_pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "  姿态: roll=%.3f, pitch=%.3f, yaw=%.3f (弧度)",
                        roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "  姿态: roll=%.1f°, pitch=%.1f°, yaw=%.1f° (角度)",
                        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "获取当前位姿失败: %s", e.what());
        }
    }

    // 将RPY角度转换为四元数
    geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
    {
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        return tf2::toMsg(quat);
    }

    // 设置规划器
    void setPlanner(const std::string &planner_name)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup 尚未初始化!");
            return;
        }

        try
        {
            move_group_->setPlannerId(planner_name);
            RCLCPP_INFO(this->get_logger(), "规划器已设置为: %s", planner_name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "设置规划器失败: %s", e.what());
        }
    }

    // 显示可用的规划器列表
    void showAvailablePlanners()
    {
        RCLCPP_INFO(this->get_logger(), "推荐的规划器列表:");
        RCLCPP_INFO(this->get_logger(), "1. RRTstar - 质量最好，速度较慢，适合复杂环境");
        RCLCPP_INFO(this->get_logger(), "2. RRTConnect - 速度快，质量一般，适合简单环境");
        RCLCPP_INFO(this->get_logger(), "3. PRM - 概率路径图，适合多次查询");
        RCLCPP_INFO(this->get_logger(), "4. PRMstar - PRM的改进版本，质量更好");
        RCLCPP_INFO(this->get_logger(), "5. BKPIECE - 双向运动树，平衡速度和质量");
        RCLCPP_INFO(this->get_logger(), "6. LBKPIECE - BKPIECE的懒惰版本");
        RCLCPP_INFO(this->get_logger(), "7. EST - 快速探索随机树");
        RCLCPP_INFO(this->get_logger(), "8. TRRT - 过渡随机树，考虑路径代价");
        std::cout << "\n当前规划器: " << move_group_->getPlannerId() << std::endl;
    }

    // 移动到指定的xyz位置和rpy姿态，支持速度和加速度控制
    bool moveToXYZRPY(double x, double y, double z, double roll, double pitch, double yaw,
                      bool is_degrees = false, double velocity_scaling = 0.1, double acceleration_scaling = 0.1)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup 尚未初始化!");
            return false;
        }

        // 如果输入是角度，转换为弧度
        if (is_degrees)
        {
            roll = roll * M_PI / 180.0;
            pitch = pitch * M_PI / 180.0;
            yaw = yaw * M_PI / 180.0;
        }

        RCLCPP_INFO(this->get_logger(), "目标位姿:");
        RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f, y=%.3f, z=%.3f", x, y, z);
        RCLCPP_INFO(this->get_logger(), "  姿态: roll=%.3f, pitch=%.3f, yaw=%.3f (弧度)", roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "  姿态: roll=%.1f°, pitch=%.1f°, yaw=%.1f° (角度)",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  速度缩放: %.2f, 加速度缩放: %.2f", velocity_scaling, acceleration_scaling);

        // 创建目标位姿
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation = rpyToQuaternion(roll, pitch, yaw);

        // 清除之前的目标和约束
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        // 确保起始状态是当前状态
        RCLCPP_INFO(this->get_logger(), "设置起始状态为当前状态...");
        move_group_->setStartStateToCurrentState();

        // 设置目标位姿，指定坐标系
        RCLCPP_INFO(this->get_logger(), "设置目标位姿...");
        move_group_->setPoseTarget(target_pose, "link6");
        

        // 设置速度和加速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_scaling);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

        // 验证目标位姿
        RCLCPP_INFO(this->get_logger(), "目标位姿已设置在坐标系: %s", move_group_->getPoseReferenceFrame().c_str());

        // 规划路径
        RCLCPP_INFO(this->get_logger(), "开始规划路径...");
        RCLCPP_INFO(this->get_logger(), "使用规划器: %s", move_group_->getPlannerId().c_str());

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        auto error_code = move_group_->plan(my_plan);

        // 详细的错误码分析
        RCLCPP_INFO(this->get_logger(), "规划结果错误码: %d", error_code.val);

        std::string error_description;
        switch (error_code.val)
        {
        case moveit::core::MoveItErrorCode::SUCCESS:
            error_description = "SUCCESS";
            break;
        case moveit::core::MoveItErrorCode::PLANNING_FAILED:
            error_description = "PLANNING_FAILED";
            break;
        case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
            error_description = "INVALID_MOTION_PLAN";
            break;
        case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            error_description = "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
            break;
        case moveit::core::MoveItErrorCode::CONTROL_FAILED:
            error_description = "CONTROL_FAILED";
            break;
        case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
            error_description = "UNABLE_TO_AQUIRE_SENSOR_DATA";
            break;
        case moveit::core::MoveItErrorCode::TIMED_OUT:
            error_description = "TIMED_OUT";
            break;
        case moveit::core::MoveItErrorCode::PREEMPTED:
            error_description = "PREEMPTED";
            break;
        case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
            error_description = "START_STATE_IN_COLLISION";
            break;
        case moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
            error_description = "START_STATE_VIOLATES_PATH_CONSTRAINTS";
            break;
        case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
            error_description = "GOAL_IN_COLLISION";
            break;
        case moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
            error_description = "GOAL_VIOLATES_PATH_CONSTRAINTS";
            break;
        case moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
            error_description = "GOAL_CONSTRAINTS_VIOLATED";
            break;
        case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
            error_description = "INVALID_GROUP_NAME";
            break;
        case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
            error_description = "INVALID_GOAL_CONSTRAINTS";
            break;
        case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
            error_description = "INVALID_ROBOT_STATE";
            break;
        case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
            error_description = "INVALID_LINK_NAME";
            break;
        case moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME:
            error_description = "INVALID_OBJECT_NAME";
            break;
        case moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
            error_description = "FRAME_TRANSFORM_FAILURE";
            break;
        case moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
            error_description = "COLLISION_CHECKING_UNAVAILABLE";
            break;
        case moveit::core::MoveItErrorCode::ROBOT_STATE_STALE:
            error_description = "ROBOT_STATE_STALE";
            break;
        case moveit::core::MoveItErrorCode::SENSOR_INFO_STALE:
            error_description = "SENSOR_INFO_STALE";
            break;
        case moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE:
            error_description = "COMMUNICATION_FAILURE";
            break;
        case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
            error_description = "NO_IK_SOLUTION";
            break;
        default:
            error_description = "UNKNOWN_ERROR";
            break;
        }

        RCLCPP_INFO(this->get_logger(), "错误描述: %s", error_description.c_str());

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "路径规划成功! 轨迹包含 %zu 个路径点",
                        my_plan.trajectory_.joint_trajectory.points.size());

            // 执行运动
            RCLCPP_INFO(this->get_logger(), "开始执行运动...");
            auto execute_result = move_group_->execute(my_plan);

            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "运动执行成功!");
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "运动执行失败! 错误代码: %d", execute_result.val);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "路径规划失败! 错误代码: %d (%s)", error_code.val, error_description.c_str());

            // 尝试不同的规划器
            std::vector<std::string> backup_planners = {"BKPIECE", "RRTstar", "PRMstar", "EST", "PRM", "TRRT"};
            std::string original_planner = move_group_->getPlannerId();

            for (const auto &planner : backup_planners)
            {
                if (planner == original_planner)
                    continue; // 跳过当前规划器

                RCLCPP_INFO(this->get_logger(), "尝试使用规划器: %s", planner.c_str());
                move_group_->setPlannerId(planner);

                auto start_time = std::chrono::system_clock::now();
                auto backup_error_code = move_group_->plan(my_plan);
                auto end_time = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end_time - start_time;

                RCLCPP_INFO(this->get_logger(), "规划用时: %.3f秒",
                            elapsed_seconds.count());
                if (backup_error_code == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(), "使用规划器 %s 规划成功!", planner.c_str());

                    // 执行运动
                    RCLCPP_INFO(this->get_logger(), "开始执行运动...");
                    auto execute_result = move_group_->execute(my_plan);

                    // 恢复原始规划器
                    move_group_->setPlannerId(original_planner);

                    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "运动执行成功!");
                        return true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "运动执行失败! 错误代码: %d", execute_result.val);
                        return false;
                    }
                }
            }

            // 恢复原始规划器
            move_group_->setPlannerId(original_planner);
            RCLCPP_WARN(this->get_logger(), "所有规划器都失败了，尝试逆运动学检查...");

            // // 尝试用默认状态重新初始化
            // RCLCPP_INFO(this->get_logger(), "尝试设置默认状态并重新规划...");
            // std::vector<double> default_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0};
            // move_group_->setJointValueTarget(default_joint_values);
            // move_group_->setStartState(*move_group_->getCurrentState());

            // // 尝试逆运动学检查（使用默认状态）
            // RCLCPP_INFO(this->get_logger(), "检查逆运动学可行性...");
            // try
            // {
            //     auto robot_model = move_group_->getRobotModel();
            //     auto kinematic_state = std::make_shared<moveit::core::RobotState>(robot_model);
            //     kinematic_state->setToDefaultValues();

            //     const moveit::core::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup("marm_group");
            //     bool ik_result = kinematic_state->setFromIK(joint_model_group, target_pose, 10.0); // 10秒超时

            //     if (ik_result)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "逆运动学求解成功，目标位姿可达");
            //         std::vector<double> joint_values;
            //         kinematic_state->copyJointGroupPositions("marm_group", joint_values);
            //         RCLCPP_INFO(this->get_logger(), "目标关节角度:");
            //         for (size_t i = 0; i < joint_values.size(); ++i)
            //         {
            //             RCLCPP_INFO(this->get_logger(), "  joint%zu: %.3f rad (%.1f°)",
            //                         i + 1, joint_values[i], joint_values[i] * 180.0 / M_PI);
            //         }

            //         // 尝试直接使用关节角度规划
            //         RCLCPP_INFO(this->get_logger(), "尝试使用关节角度直接规划...");
            //         move_group_->setJointValueTarget(joint_values);
            //         auto joint_plan_result = move_group_->plan(my_plan);

            //         if (joint_plan_result == moveit::core::MoveItErrorCode::SUCCESS)
            //         {
            //             RCLCPP_INFO(this->get_logger(), "关节角度规划成功!");
            //             // 可以选择执行这个规划
            //             // auto execute_result = move_group_->execute(my_plan);
            //             return true; // 暂时返回成功，实际可以执行
            //         }
            //         else
            //         {
            //             RCLCPP_ERROR(this->get_logger(), "关节角度规划也失败了，错误码: %d", joint_plan_result.val);
            //         }
            //     }
            //     else
            //     {
            //         RCLCPP_ERROR(this->get_logger(), "逆运动学求解失败，目标位姿不可达!");
            //     }
            // }
            // catch (const std::exception &e)
            // {
            //     RCLCPP_ERROR(this->get_logger(), "逆运动学检查异常: %s", e.what());
            // }

            return false;
        }
    }

    // 移动到预定义位置，支持速度和加速度控制
    bool moveToHome(double velocity_scaling = 0.1, double acceleration_scaling = 0.1)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup 尚未初始化!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "移动到stand位置...");
        RCLCPP_INFO(this->get_logger(), "速度缩放: %.2f, 加速度缩放: %.2f", velocity_scaling, acceleration_scaling);

        // 清除之前的目标和约束
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();

        // 设置起始状态为当前状态
        move_group_->setStartStateToCurrentState();

        // 设置速度和加速度缩放因子
        move_group_->setMaxVelocityScalingFactor(velocity_scaling);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

        // 尝试使用命名目标
        try
        {
            move_group_->setNamedTarget("stand");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "命名目标'stand'不存在，使用默认关节值: %s", e.what());
            // 使用默认关节值
            std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0};
            move_group_->setJointValueTarget(home_joint_values);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto error_code = move_group_->plan(my_plan);

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "stand位置规划成功!");
            auto execute_result = move_group_->execute(my_plan);

            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "移动到stand位置成功!");
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "移动到stand位置执行失败! 错误代码: %d", execute_result.val);
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "stand位置规划失败! 错误代码: %d", error_code.val);
            return false;
        }
    }

    // 交互式控制模式
    void interactiveMode()
    {
        // 等待初始化完成
        while (!initialized_ && rclcpp::ok())
        {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!initialized_)
        {
            RCLCPP_ERROR(this->get_logger(), "初始化失败!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "=== 进入交互式控制模式 ===");
        RCLCPP_INFO(this->get_logger(), "输入目标位姿，输入 'q' 退出");

        // 默认速度和加速度缩放因子
        double current_velocity_scaling = 0.1;
        double current_acceleration_scaling = 0.1;

        while (rclcpp::ok())
        {
            std::cout << "\n当前设置 - 速度缩放: " << current_velocity_scaling
                      << ", 加速度缩放: " << current_acceleration_scaling
                      << ", 规划器: " << move_group_->getPlannerId() << std::endl;
            std::cout << "请选择操作:\n";
            std::cout << "1. 输入目标位置和姿态 (格式: x y z roll pitch yaw)\n";
            std::cout << "2. 设置速度缩放因子 (格式: vel 0.1-1.0)\n";
            std::cout << "3. 设置加速度缩放因子 (格式: acc 0.1-1.0)\n";
            std::cout << "4. 设置规划器 (格式: planner RRTstar)\n";
            std::cout << "5. 查看可用规划器 (输入: planners)\n";
            std::cout << "6. 返回Home位置 (输入: home)\n";
            std::cout << "7. 查看当前位姿 (输入: current)\n";
            std::cout << "8. 退出 (输入: q)\n";
            std::cout << "请输入: ";

            std::string input;
            std::getline(std::cin, input);

            if (input == "q" || input == "quit")
            {
                break;
            }

            if (input == "home")
            {
                moveToHome(current_velocity_scaling, current_acceleration_scaling);
                continue;
            }

            if (input == "current")
            {
                showCurrentPose();
                continue;
            }

            if (input == "planners")
            {
                showAvailablePlanners();
                continue;
            }

            // 检查是否是规划器设置命令
            if (input.substr(0, 7) == "planner")
            {
                std::istringstream iss(input.substr(8));
                std::string planner_name;
                if (iss >> planner_name)
                {
                    setPlanner(planner_name);
                }
                else
                {
                    std::cout << "请指定规划器名称，例如: planner RRTstar" << std::endl;
                    showAvailablePlanners();
                }
                continue;
            }

            // 检查是否是速度设置命令
            if (input.substr(0, 3) == "vel")
            {
                std::istringstream iss(input.substr(4));
                double vel;
                if (iss >> vel && vel >= 0.01 && vel <= 1.0)
                {
                    current_velocity_scaling = vel;
                    std::cout << "速度缩放因子已设置为: " << current_velocity_scaling << std::endl;
                }
                else
                {
                    std::cout << "无效的速度值! 请输入0.01-1.0之间的值" << std::endl;
                }
                continue;
            }

            // 检查是否是加速度设置命令
            if (input.substr(0, 3) == "acc")
            {
                std::istringstream iss(input.substr(4));
                double acc;
                if (iss >> acc && acc >= 0.01 && acc <= 1.0)
                {
                    current_acceleration_scaling = acc;
                    std::cout << "加速度缩放因子已设置为: " << current_acceleration_scaling << std::endl;
                }
                else
                {
                    std::cout << "无效的加速度值! 请输入0.01-1.0之间的值" << std::endl;
                }
                continue;
            }

            // 解析位置和姿态输入
            std::istringstream iss(input);
            double x, y, z, roll, pitch, yaw;

            if (iss >> x >> y >> z >> roll >> pitch >> yaw)
            {
                moveToXYZRPY(x, y, z, roll, pitch, yaw, true,
                             current_velocity_scaling, current_acceleration_scaling); // 使用角度和当前缩放因子
            }
            else
            {
                std::cout << "输入格式错误! 请按照以下格式输入:\n";
                std::cout << "位置和姿态: x y z roll pitch yaw (角度)\n";
                std::cout << "设置速度: vel 0.1-1.0\n";
                std::cout << "设置加速度: acc 0.1-1.0\n";
                std::cout << "设置规划器: planner RRTstar\n";
                std::cout << "其他命令: home, current, planners, q\n";
            }
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    bool initialized_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建节点
    auto demo_node = std::make_shared<MoveItDemo>();

    std::cout << "=== MoveIt 机械臂控制演示 ===\n";
    std::cout << "支持xyz位置和rpy姿态控制，可调节速度和加速度\n";
    std::cout << "正在启动交互式控制模式...\n\n";

    // 直接进入交互式控制模式
    demo_node->interactiveMode();

    rclcpp::shutdown();
    return 0;
}
