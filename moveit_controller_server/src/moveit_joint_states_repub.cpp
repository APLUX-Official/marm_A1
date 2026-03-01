#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

class JointStateReorderer : public rclcpp::Node {
  public:
    JointStateReorderer() : Node("joint_state_reorderer") {
        // 定义目标关节顺序 (按照您要求的顺序)
        target_joint_order_ = {
            "idx13_left_arm_joint1",  "idx14_left_arm_joint2",
            "idx15_left_arm_joint3",  "idx16_left_arm_joint4",
            "idx17_left_arm_joint5",  "idx18_left_arm_joint6",
            "idx19_left_arm_joint7",  "idx20_right_arm_joint1",
            "idx21_right_arm_joint2", "idx22_right_arm_joint3",
            "idx23_right_arm_joint4", "idx24_right_arm_joint5",
            "idx25_right_arm_joint6", "idx26_right_arm_joint7"};

        // 订阅原始 joint_states
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointStateReorderer::joint_state_callback, this,
                      std::placeholders::_1));

        // 发布重新排序后的 joint_states
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states_rec", 10);

        publisher_command_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/motion/control/arm_joint_command", 10);

        RCLCPP_INFO(this->get_logger(), "Joint State Reorderer 已启动");
    }

  private:
    void
    joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto reordered_msg = std::make_shared<sensor_msgs::msg::JointState>();
        reordered_msg->header = msg->header;

        // 保持目标关节顺序
        reordered_msg->name = target_joint_order_;
        reordered_msg->position.resize(target_joint_order_.size(), 0.0);
        reordered_msg->velocity.resize(target_joint_order_.size(), 0.0);
        reordered_msg->effort.resize(target_joint_order_.size(), 0.0);

        // 为每个目标关节查找原始消息中的对应位置
        for (size_t i = 0; i < target_joint_order_.size(); ++i) {
            const auto &joint_name = target_joint_order_[i];

            // 在原始消息中查找当前关节
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
            if (it != msg->name.end()) {
                // 找到关节在原始消息中的索引
                size_t original_index = std::distance(msg->name.begin(), it);

                // 复制position数据到新位置
                if (original_index < msg->position.size()) {
                    reordered_msg->position[i] = msg->position[original_index];
                }

                // 复制velocity和effort数据(如果存在)
                if (original_index < msg->velocity.size()) {
                    reordered_msg->velocity[i] = msg->velocity[original_index];
                }
                if (original_index < msg->effort.size()) {
                    reordered_msg->effort[i] = msg->effort[original_index];
                }
            } else {
                // 未找到该关节，记录警告并使用默认值0.0
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                     5000, // 每5秒最多警告一次
                                     "未找到关节: %s", joint_name.c_str());
            }
        }

        // 发布重新排序后的消息
        publisher_->publish(*reordered_msg);
        publisher_command_->publish(*reordered_msg);
    }

    std::vector<std::string> target_joint_order_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        publisher_command_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateReorderer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}