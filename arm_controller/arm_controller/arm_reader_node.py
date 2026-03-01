import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import threading

# 从之前的代码中使用的函数
def calculate_angle(pwm, angle_min=-135, angle_max=135, pwm_min=500, pwm_max=2500):
    angle = angle_min + (pwm - pwm_min) * (angle_max - angle_min) / (pwm_max - pwm_min)
    return max(min(angle, angle_max), angle_min)

def read_servo_position(ser, servo_id):
    """
    读取指定舵机的当前位置角度，并测量读取时间。
    :param ser: serial.Serial 对象，用于串口通信
    :param servo_id: int, 舵机 ID（范围 0-254）
    :return: float, 舵机的当前位置角度
    """
    id_str = f"{servo_id:03d}"
    command = f"#{id_str}PRAD!"
    ser.write(command.encode())

    # 等待舵机的返回数据
    response = ser.readline().decode().strip()

    # 解析返回的PWM值
    if response.startswith(f"#{id_str}P"):
        pwm_str = response[5:9]
        pwm = int(pwm_str)
        angle = calculate_angle(pwm)
        return angle
    else:
        print(f"无效的返回数据: {response}")
        return None

def send_unload_command(ser, servo_id):
    """
    发送舵机卸力指令，使舵机停止受力。
    :param ser: serial.Serial 对象，用于串口通信
    :param servo_id: int, 舵机 ID（范围 0-254）
    """
    id_str = f"{servo_id:03d}"
    command = f"#{id_str}PULK!"
    ser.write(command.encode())

# ROS2节点类，用于读取舵机角度并发布
class ArmReaderNode(Node):

    def __init__(self):
        super().__init__('arm_reader_node')

        # 创建发布器，发布 Float64MultiArray 类型的消息
        self.publisher_ = self.create_publisher(Float64MultiArray, 'servo_angles', 10)

        # 设定定时器，每100毫秒读取一次舵机角度并发布
        self.timer_period = 0.05  # 单位为秒 (100ms)
        self.timer = None  # 初始化定时器为空，稍后设置

        # 打开串口连接，设置波特率为115200，超时时间为0.1秒
        self.ser = serial.Serial(port='/dev/ttyCH341USB0', baudrate=115200, timeout=0.1)
        self.lock = threading.Lock()  # 线程锁，用于串口访问控制

        self.get_logger().info('Arm Reader Node has started.')

        # 启动时先卸力
        self.send_unload_commands()

        # 卸力完成后，启动读取数据的定时器
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def send_unload_commands(self):
        """
        启动时发送卸力指令到所有舵机。
        """
        self.get_logger().info('Sending unload commands to all servos.')
        for i in range(6):  # 舵机 ID 从 0 到 5
            send_unload_command(self.ser, i)
            time.sleep(0.1)  # 等待一会儿，以确保卸力指令被处理

    def read_angle_for_servo(self, servo_id):
        """
        在线程中读取指定舵机的角度值。
        """
        with self.lock:
            return read_servo_position(self.ser, servo_id)

    def timer_callback(self):
        """
        定时器的回调函数，用于每隔一段时间读取舵机的角度并发布。
        """
        angles = []

        # 使用多线程读取每个舵机的角度
        threads = []
        results = [None] * 6

        for i in range(6):  # 舵机 ID 从 0 到 5
            thread = threading.Thread(target=lambda idx=i: results.__setitem__(idx, self.read_angle_for_servo(idx)))
            threads.append(thread)
            thread.start()

        # 等待所有线程完成
        for thread in threads:
            thread.join()

        # 收集结果
        for result in results:
            if result is not None:
                angles.append(result)
            else:
                self.get_logger().warn("Failed to read angle.")
                angles.append(0.0)  # 读取失败时，设置角度为0

        # 创建并发布消息
        angle_msg = Float64MultiArray()
        angle_msg.data = angles
        self.publisher_.publish(angle_msg)

        self.get_logger().info(f'Published angles: {angles}')

    def destroy_node(self):
        self.ser.close()  # 关闭串口
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # 创建并启动ArmReaderNode节点
    arm_reader_node = ArmReaderNode()

    try:
        rclpy.spin(arm_reader_node)
    except KeyboardInterrupt:
        arm_reader_node.get_logger().info('Shutting down Arm Reader Node.')
    finally:
        arm_reader_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
