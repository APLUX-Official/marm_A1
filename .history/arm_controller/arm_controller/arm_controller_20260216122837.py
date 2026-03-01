import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
import serial
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

# 前面定义的函数
def calculate_pwm(angle, angle_min=-135, angle_max=135, pwm_min=500, pwm_max=2500):
    pwm = pwm_min + int(((angle - angle_min) * (pwm_max - pwm_min)) / (angle_max - angle_min))
    return max(min(pwm, pwm_max), pwm_min)

def send_servo_command(ser, servo_id, angle, time):
    id_str = f"{servo_id:03d}"
    pwm = calculate_pwm(angle)
    pwm_str = f"P{pwm:04d}"
    time_str = f"T{time:04d}"
    command = f"#{id_str}{pwm_str}{time_str}!"
    print(f"发送指令: {command}")
    ser.write(command.encode())

# ROS2节点类，用于控制6个舵机
class ArmControllerNode(Node):

    def __init__(self):
        super().__init__('arm_controller_node')

        # 订阅 Float64MultiArray 类型的消息
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'arm_angles',  # 话题名称
            self.listener_callback,
            10  # 队列大小
        )
        
        # 订阅 joint_states 话题
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',  # joint_states话题名称
            self.joint_states_callback,
            10  # 队列大小
        )
        
        # 订阅 Bool 类型消息来控制关节6
        self.gripper_subscription = self.create_subscription(
            Bool,
            'gripper_control',  # 话题名称
            self.gripper_callback,
            10  # 队列大小
        )
        
        # 打开串口连接，设置波特率为115200
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

        # 读取舵机偏差补偿配置文件
        self.servo_offsets = self.load_servo_offsets()
        self.get_logger().info(f'Loaded servo offsets: {self.servo_offsets}')

        self.get_logger().info('Arm Controller Node has started.')

    def load_servo_offsets(self):
        """
        从YAML文件加载舵机角度偏差补偿配置。
        
        :return: 包含6个舵机偏差值的字典
        """
        try:
            # 尝试从包的config目录加载配置文件
            package_share_dir = get_package_share_directory('arm_controller')
            config_file = os.path.join(package_share_dir, 'config', 'servo_calibration.yaml')
        except:
            # 如果包路径获取失败，尝试相对路径
            config_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'servo_calibration.yaml')
        
        # 如果找不到配置文件，返回默认值（全0）
        if not os.path.exists(config_file):
            self.get_logger().warn(f'Servo calibration file not found at {config_file}, using zero offsets.')
            return {f'servo_{i}': 0.0 for i in range(6)}
        
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                offsets = config.get('servo_offsets', {})
                # 确保所有6个舵机都有偏差值
                return {f'servo_{i}': offsets.get(f'servo_{i}', 0.0) for i in range(6)}
        except Exception as e:
            self.get_logger().error(f'Error loading servo calibration file: {e}')
            return {f'servo_{i}': 0.0 for i in range(6)}
    
    def apply_offset(self, servo_id, angle):
        """
        应用舵机角度偏差补偿。
        
        :param servo_id: 舵机ID (0-5)
        :param angle: 原始角度
        :return: 补偿后的角度
        """
        offset = self.servo_offsets.get(f'servo_{servo_id}', 0.0)
        return angle + offset

    def listener_callback(self, msg):
        """
        回调函数，处理接收到的消息并发送舵机控制指令。
        
        :param msg: std_msgs/msg/Float64MultiArray 消息，包含六个舵机的角度值
        """
        angles = msg.data  # 获取传入的6个舵机角度
        if len(angles) != 6:
            self.get_logger().error("Received incorrect number of angles, expected 6.")
            return

        run_time = 0  # 设定每个舵机的运行时间为 1000ms

        # 发送控制指令到每个舵机
        for i in range(6):  # i 对应舵机 ID 0 到 5
            angle = angles[i]
            # 应用角度偏差补偿
            compensated_angle = self.apply_offset(i, angle)
            send_servo_command(self.ser, i, compensated_angle, run_time)
            time.sleep(0.1)  # 增加一点延时，确保每个指令都能被处理
        
        self.get_logger().info(f'Sent commands to all servos: {angles}')

    def joint_states_callback(self, msg):
        """
        joint_states话题的回调函数，处理接收到的关节状态消息并控制机械臂。
        
        :param msg: sensor_msgs/msg/JointState 消息，包含关节名称、位置、速度和力矩信息
        """
        # 定义期望的关节顺序
        expected_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 检查是否有足够的关节数据
        if len(msg.name) < 5:
            self.get_logger().warn(f"Received joint_states with {len(msg.name)} joints, expected at least 5.")
            return
        
        # 创建关节名称到位置的映射
        joint_position_map = {}
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                joint_position_map[joint_name] = msg.position[i]
        
        # 按照期望顺序重新排列关节角度，并转换为度数
        ordered_angles = []
        for joint_name in expected_joints[:5]:  # 只处理前5个关节
            if joint_name in joint_position_map:
                angle_rad = joint_position_map[joint_name]
                angle_deg = angle_rad * 180.0 / 3.14159
                # 限制角度范围在-135到135度之间
                angle_deg = max(min(angle_deg, 135), -135)
                ordered_angles.append(angle_deg)
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in joint_states message.")
                return
        
        # 如果只有5个关节，不需要添加第6个舵机的默认值
        # 第6个舵机（夹爪）由gripper_callback单独控制
        
        # 记录接收到的关节状态（按原始顺序）
        original_angles = [pos * 180.0 / 3.14159 for pos in msg.position[:len(msg.name)]]
        # self.get_logger().info(f'Received joint_states: {msg.name} with positions: {original_angles}')
        # self.get_logger().info(f'Reordered for servos (joint1-5): {ordered_angles}')
        
        run_time = 200  # 设定每个舵机的运行时间为50ms
        
        # 只发送控制指令到前5个舵机，不控制第6个舵机（夹爪）
        for i in range(5):  # i 对应舵机 ID 0 到 4（joint1-5）
            angle = ordered_angles[i]
            # 应用角度偏差补偿
            compensated_angle = self.apply_offset(i, angle)
            send_servo_command(self.ser, i, compensated_angle, run_time)
            time.sleep(0.005)  # 增加一点延时，确保每个指令都能被处理
        
        # self.get_logger().info(f'Sent joint_states commands to servos 0-4: {ordered_angles}')

    def gripper_callback(self, msg):
        """
        夹爪控制回调函数，根据接收到的Bool消息控制关节6（夹爪）。
        
        :param msg: std_msgs/msg/Bool 消息，true表示关闭夹爪(-50度)，false表示打开夹爪(50度)
        """
        # 根据Bool值设置关节6的角度
        if msg.data:
            angle = -50.0  # True: open夹爪，设置为-50度
            action = "打开"
        else:
            angle = 5.0   # False: close夹爪，设置为10度
            action = "关闭"
        
        run_time = 500  # 设定舵机运行时间为500ms
        
        # 应用角度偏差补偿
        compensated_angle = self.apply_offset(5, angle)
        
        # 发送控制指令到关节6（舵机ID 5）
        send_servo_command(self.ser, 5, compensated_angle, run_time)
        
        self.get_logger().info(f'Gripper control: {action} - Set joint6 to {angle} degrees (compensated: {compensated_angle})')

    def destroy_node(self):
        self.ser.close()  # 关闭串口
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # 创建并启动ArmControllerNode节点
    arm_controller_node = ArmControllerNode()

    try:
        rclpy.spin(arm_controller_node)
    except KeyboardInterrupt:
        arm_controller_node.get_logger().info('Shutting down Arm Controller Node.')
    finally:
        arm_controller_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
