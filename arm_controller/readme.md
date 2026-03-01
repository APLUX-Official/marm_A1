## joint-states机械臂控制
该节点调用joint-states接口，moveit规划的各个关节角度会通过joint-states节点发出，arm-control节点监听joint-states数据，并根据舵机驱动，转化成串口数据发布给机械臂

## 家爪开和控制

**打开家爪**
ros2 topic pub /gripper_control std_msgs/msg/Bool "data: true" --once
**关闭家爪**
ros2 topic pub /gripper_control std_msgs/msg/Bool "data: false" --once