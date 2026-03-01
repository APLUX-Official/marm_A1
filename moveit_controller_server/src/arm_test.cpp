#include "custom_interface/srv/set_arm_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>

class ArmTestNode : public rclcpp::Node
{
public:
    ArmTestNode() : Node("arm_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "机械臂测试节点已启动");

        // 创建服务客户端
        client_ = this->create_client<custom_interface::srv::SetArmAction>(
            "set_arm_action");

        // 等待服务可用
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务时被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待 set_arm_action 服务可用...");
        }

        RCLCPP_INFO(this->get_logger(), "服务已连接，开始执行动作序列");

        // 开始执行动作循环
        run_action_sequence();
    }

private:
    rclcpp::Client<custom_interface::srv::SetArmAction>::SharedPtr client_;

    // 调用服务执行单个动作
    bool execute_action(const std::string &arm_name, const std::string &action_name)
    {
        auto request = std::make_shared<custom_interface::srv::SetArmAction::Request>();
        request->arm_name = arm_name;
        request->arm_action = action_name;

        RCLCPP_INFO(this->get_logger(), "执行动作: %s -> %s", 
                    arm_name.c_str(), action_name.c_str());

        auto result_future = client_->async_send_request(request);

        // 等待结果
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), 
                                                result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            if (result->arm_status)
            {
                RCLCPP_INFO(this->get_logger(), "动作 %s 执行成功: %s",
                            action_name.c_str(), result->message.c_str());
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "动作 %s 执行失败: %s",
                             action_name.c_str(), result->message.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
            return false;
        }
    }

    // 执行动作序列
    void run_action_sequence()
    {
        // 定义机械臂名称（根据你的配置修改）
        std::string arm_name = "marm_group";  // 修改为你的机械臂组名称

        // 定义动作序列
        std::vector<std::string> actions = {"home", "stand", "get"};

        while (rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), 
                        "\n======== 开始新的动作循环 ========");

            // 依次执行 home -> stand -> get
            bool all_success = true;
            for (const auto &action : actions)
            {
                if (!execute_action(arm_name, action))
                {
                    RCLCPP_ERROR(this->get_logger(), "动作序列执行失败，停止循环");
                    all_success = false;
                    break;
                }
            }

            if (!all_success)
            {
                break;
            }

            RCLCPP_INFO(this->get_logger(), 
                        "======== 动作循环完成，延时1秒 ========\n");

            // 延时1秒
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmTestNode>();

    RCLCPP_INFO(node->get_logger(), "节点退出");
    rclcpp::shutdown();
    return 0;
}
