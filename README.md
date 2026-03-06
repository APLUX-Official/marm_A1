# Design and Implementation of MARM Manipulator Control System Based on ROS 2 and Machine Vision

## 1. Robot Design Philosophy and Application Scenarios

### 1.1 Design Philosophy

The MARM (Mobile Arm / Manipulator) manipulator project aims to build an open-source, modular, and easily extensible desktop-level 6-axis manipulator control system based on ROS 2 (Humble). The system integrates machine vision (OpenCV), motion planning (MoveIt 2), and low-level hardware control to form a complete "perception-planning-execution" closed loop. In terms of design, the project emphasizes hardware-software decoupling, and achieves efficient collaboration among various modules through standard ROS 2 interfaces (such as `JointState` and the custom `SetArmAction`).

### 1.2 Application Scenarios

- **Education and Research**: Serves as an ideal platform for learning ROS 2, robot kinematics, machine vision, and hand-eye calibration.
- **Automated Sorting**: Combines visual recognition to realize automatic grasping and sorting of objects with specific colors and shapes.
- **Flexible Manufacturing Verification**: Used to verify complex grasping algorithms and multi-sensor fusion schemes.

## 2. Hardware Selection and System Environment Setup

### 2.1 Hardware Selection

The hardware components required for this project are as follows:

- **Manipulator Body**: A1 6-degree-of-freedom (DOF) desktop-level manipulator, adopting a combination of 3D-printed shell and metal structural parts.
- **Drive Actuators**: 6 bus/PWM servos, responsible for controlling each joint and end gripper of the manipulator.
- **Core Controller**: A host computer (such as a PC or Raspberry Pi) running ROS 2, communicating with the low-level servo control board through a USB-to-serial module.
- **Visual Sensor**: RGB monocular camera (such as a USB camera), used for target detection and pose estimation.
- **Calibration Board**: Aruco QR code or checkerboard calibration board, used for hand-eye calibration.

### 2.2 System Environment Setup

- **Operating System**: Ubuntu 22.04 LTS
- **Core Framework**: ROS 2 Humble
- **Key Dependencies**:
  - MoveIt 2 (Motion planning and inverse kinematics solution)
  - OpenCV & cv_bridge (Image processing and visual recognition)
  - tf2_ros (Coordinate transformation and TF tree management)
  - easy_handeye2 (Hand-eye calibration toolkit)

## 3. Core Algorithm Analysis and Code Structure Description

### 3.1 Code Structure Description

The project adopts the standard modular workspace structure of ROS 2, mainly including the following functional packages:

- `A1_description` / `A1_moveit_config`: Contains the URDF model, SRDF semantic description of the manipulator, as well as kinematics and controller configuration files for MoveIt 2.
- `arm_controller`: A low-level hardware driver node written in Python, responsible for converting ROS 2 joint angle commands into serial protocols and sending them to servos, and supports servo zero offset compensation.
- `capture_image`: A visual perception node written in C++, including RGB color recognition (`rgb_detect_pose_node`) and Aruco code recognition (`aruco_object_pose_node`) based on OpenCV.
- `easy_handeye2`: A hand-eye calibration toolkit implemented in C++, supporting multiple calibration algorithms (such as Tsai-Lenz, Park, etc.).
- `custom_interface`: Custom ROS 2 service and action interfaces (e.g., `SetArmAction.srv`).

### 3.2 Blog Content: Detailed Explanation of Core Algorithms

#### 3.2.1 Detailed Explanation of Hand-Eye Calibration Process (Eye-on-Base vs Eye-in-Hand) and Source Code Analysis

Hand-Eye Calibration is the key to realizing visual grasping, aiming to solve the spatial transformation relationship between the camera coordinate system and the manipulator coordinate system. This project implements two classic calibration modes through the `easy_handeye2` package, and its core logic is divided into two parts: **TF data sampling** and **OpenCV matrix solution**.

**1. TF Data Sampling Logic (`handeye_sampler.cpp`)**
During the calibration process, the system needs to record multiple sets of manipulator poses and the calibration board poses recognized by the camera. `easy_handeye2` obtains these transformation matrices by listening to the ROS 2 TF tree. The sampling logic varies for different installation modes:

```cpp
// Core code snippet: Obtain manipulator pose according to calibration mode (handeye_sampler.cpp)
if (params_.eye_on_hand) {
  // Eye-in-Hand: Obtain the transform from base -> effector
  sample.robot_transform = tf_buffer_->lookupTransform(
    params_.robot_base_frame,
    params_.robot_effector_frame,
    tf2::TimePointZero,
    tf2::durationFromSec(5.0));
} else {
  // Eye-on-Base: Obtain the transform from effector -> base
  // Note: To unify the AX=XB solution model, the TF transform direction is reversed here
  sample.robot_transform = tf_buffer_->lookupTransform(
    params_.robot_effector_frame,
    params_.robot_base_frame,
    tf2::TimePointZero,
    tf2::durationFromSec(5.0));
}

// Obtain camera visual pose: transform from camera -> marker
sample.optical_transform = tf_buffer_->lookupTransform(
  params_.tracking_base_frame,
  params_.tracking_marker_frame,
  tf2::TimePointZero,
  tf2::durationFromSec(5.0));
```

*Analysis*: For the "Eye-on-Base" mode, the $AX=XB$ equation is solved, where $A$ is the relative transform of the manipulator end-effector between different poses, and $B$ is the relative transform of the calibration board in the camera coordinate system. To unify the mathematical model, the source code intentionally reverses the direction of the manipulator TF transform for "Eye-on-Base" (i.e., `effector -> base`), thereby reusing the same set of solution algorithms.

**2. Calibration Algorithm Solution (`calibration_backend_opencv.cpp`)**
After collecting enough samples (usually 10-15 sets), `easy_handeye2` calls OpenCV's `cv::calibrateHandEye` function for solution. This backend supports multiple classic calibration algorithms:

```cpp
// Core code snippet: OpenCV calibration algorithm mapping (calibration_backend_opencv.cpp)
cv::HandEyeCalibrationMethod CalibrationBackendOpenCV::getOpenCVMethod() const
{
  switch (current_algorithm_) {
    case CalibrationAlgorithm::TSAI_LENZ:
      return cv::CALIB_HAND_EYE_TSAI;       // Tsai-Lenz algorithm (default, fast speed)
    case CalibrationAlgorithm::PARK:
      return cv::CALIB_HAND_EYE_PARK;       // Park algorithm
    case CalibrationAlgorithm::HORAUD:
      return cv::CALIB_HAND_EYE_HORAUD;     // Horaud algorithm
    case CalibrationAlgorithm::ANDREFF:
      return cv::CALIB_HAND_EYE_ANDREFF;    // Andreff algorithm
    case CalibrationAlgorithm::DANIILIDIS:
      return cv::CALIB_HAND_EYE_DANIILIDIS; // Daniilidis algorithm (dual quaternion, high precision)
    default:
      return cv::CALIB_HAND_EYE_TSAI;
  }
}
```

*Analysis*:

- **Tsai-Lenz**: Based on the axis-angle representation, solves rotation and translation separately with fast calculation speed, which is the default algorithm of the system.
- **Daniilidis**: Based on dual quaternions, solves rotation and translation together simultaneously, and can provide higher precision in the case of high noise.
  The system converts ROS `geometry_msgs::msg::Transform` into OpenCV rotation matrix (`cv::Mat`) and translation vector, passes them into the above algorithms to solve the final static TF transformation matrix (i.e., the fixed pose relationship between the camera and the manipulator), and saves it as a YAML configuration file for subsequent grasping tasks to load.

#### 3.2.2 Code Excerpt: Target Detection and Pose Estimation Based on OpenCV, Inverse Kinematics Grasp Planning

**1. Target Detection and Pose Estimation Based on OpenCV**
In the `capture_image` package, `rgb_detect_pose_node.cpp` implements color-based target detection and 3D pose estimation. The core logic is to extract target contours through color threshold filtering and estimate depth combined with camera intrinsic parameters:

```cpp
// Core code snippet: Color threshold filtering and contour extraction
cv::Mat hsv_image, mask;
cv::cvtColor(cv_image->image, hsv_image, cv::COLOR_BGR2HSV);

// Apply HSV threshold to generate mask
cv::inRange(hsv_image, 
            cv::Scalar(threshold.h_min, threshold.s_min, threshold.v_min),
            cv::Scalar(threshold.h_max, threshold.s_max, threshold.v_max), 
            mask);

// Find external contours
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// Calculate the minimum enclosing circle for the largest contour
cv::Point2f center;
float radius;
cv::minEnclosingCircle(contours[0], center, radius);

// Estimate depth (Z) combined with camera intrinsic parameters (ost.yaml) and the real physical size of the object
double focal_length = camera_matrix_.at<double>(0, 0);
double depth = (circle_real_diameter_ * focal_length) / (radius * 2.0);

// Calculate the 3D coordinates (X, Y, Z) of the object in the camera coordinate system
double x = (center.x - camera_matrix_.at<double>(0, 2)) * depth / focal_length;
double y = (center.y - camera_matrix_.at<double>(1, 2)) * depth / focal_length;
```

This node estimates the depth of the target using the principle of similar triangles through the camera intrinsic parameters and the pixel size of the target in the image, then publishes `object_pose` and broadcasts the TF transform.

**2. Inverse Kinematics Grasp Planning and Low-Level Control**
After obtaining the pose of the target in the base coordinate system, MoveIt 2 performs Inverse Kinematics (IK) solution to generate trajectories in the joint space. `arm_controller.py` is responsible for receiving these joint angles and driving the hardware:

```python
# Core code snippet: Receive joint angles and convert to servo PWM commands
def listener_callback(self, msg):
    # msg.data contains the target angles (in radians) of 6 joints planned by MoveIt
    angles = msg.data
    for i in range(6):
        # Convert radians to degrees and apply servo offset compensation from YAML configuration file
        angle_deg = math.degrees(angles[i]) + self.servo_offsets[f'servo_{i}']
        
        # Map physical angles to servo PWM signals (500-2500)
        pwm = self.calculate_pwm(angle_deg, angle_min=-135, angle_max=135)
        
        # Splice serial communication protocol commands and send to the lower computer
        # Protocol format: #<id>P<pwm>T<time>!
        self.send_servo_command(self.ser, servo_id=i, angle=angle_deg, time=1000)
```

This controller not only implements linear mapping from angles to PWM but also introduces `servo_calibration.yaml` for software-level zero calibration, effectively eliminating errors caused by mechanical assembly.

## 4. Typical Case Usage Instructions

### 4.1 Comprehensive Case of Visual Grasping

This case demonstrates how to connect the above modules in series to realize the complete process of "recognition-localization-grasping".

**Operation Steps**:

1. **Start low-level driver and MoveIt**:

   ```bash
   ros2 launch A1_moveit_config demo.launch.py
   ros2 run arm_controller arm_controller_node
   ```

2. **Start visual recognition node**:

   ```bash
   ros2 launch capture_image capture_image.launch.py
   ```

3. **Publish static hand-eye calibration TF** (assuming calibration is completed):

   ```bash
   ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll base_link camera_link
   ```

4. **Execute grasping task**:
   After the visual node recognizes the target, it will broadcast the `object` coordinate system to the TF tree. At this time, the self-written grasping action client can be used to read the transform from `base_link` to `object`, send it as the target pose to MoveIt 2 for path planning and execution.

# 基于ROS 2与机器视觉的MARM机械臂控制系统设计与实现

## 1. 机器人设计理念与应用场景

### 1.1 设计理念
MARM (Mobile Arm / Manipulator) 机械臂项目旨在构建一个基于ROS 2 (Humble) 的开源、模块化、易于扩展的桌面级六轴机械臂控制系统。系统集成了机器视觉（OpenCV）、运动规划（MoveIt 2）和底层硬件控制，形成了一个完整的“感知-规划-执行”闭环。在设计上，项目强调软硬件解耦，通过标准的ROS 2接口（如 `JointState` 和自定义的 `SetArmAction`）实现各模块的高效协同。

### 1.2 应用场景
- **教育与科研**：作为学习ROS 2、机器人运动学、机器视觉和手眼标定的理想平台。
- **自动化分拣**：结合视觉识别，实现对特定颜色、形状物体的自动抓取与分拣。
- **柔性制造验证**：用于验证复杂的抓取算法和多传感器融合方案。

## 2. 硬件选型与系统环境搭建

### 2.1 硬件选型
本项目所需的硬件组件如下：
- **机械臂本体**：A1 6自由度桌面级机械臂，采用3D打印外壳与金属结构件结合。
- **驱动执行器**：6个总线/PWM舵机，负责控制机械臂的各个关节及末端夹爪。
- **核心控制器**：运行ROS 2的上位机（如PC或树莓派），通过USB转串口模块与底层舵机控制板通信。
- **视觉传感器**：RGB单目相机（如USB摄像头），用于目标检测与位姿估计。
- **标定板**：Aruco二维码或棋盘格标定板，用于手眼标定。

### 2.2 系统环境搭建
- **操作系统**：Ubuntu 22.04 LTS
- **核心框架**：ROS 2 Humble
- **关键依赖**：
  - MoveIt 2（运动规划与逆运动学求解）
  - OpenCV & cv_bridge（图像处理与视觉识别）
  - tf2_ros（坐标变换与TF树管理）
  - easy_handeye2（手眼标定工具包）

## 3. 核心算法解析与代码结构说明

### 3.1 代码结构说明
项目采用ROS 2标准的模块化工作空间结构，主要包含以下功能包：
- `A1_description` / `A1_moveit_config`：包含机械臂的URDF模型、SRDF语义描述以及MoveIt 2的运动学和控制器配置文件。
- `arm_controller`：Python编写的底层硬件驱动节点，负责将ROS 2的关节角度指令转换为串口协议发送给舵机，并支持舵机零点偏差补偿。
- `capture_image`：C++编写的视觉感知节点，包含基于OpenCV的RGB颜色识别（`rgb_detect_pose_node`）和Aruco码识别（`aruco_object_pose_node`）。
- `easy_handeye2`：C++实现的手眼标定工具包，支持多种标定算法（如Tsai-Lenz、Park等）。
- `custom_interface`：自定义的ROS 2服务和动作接口（如 `SetArmAction.srv`）。

### 3.2 博客内容：核心算法详解

#### 3.2.1 详解眼在手外与眼在手上的手眼标定流程与源码解析
手眼标定（Hand-Eye Calibration）是实现视觉抓取的关键，旨在求解相机坐标系与机械臂坐标系之间的空间变换关系。本项目通过 `easy_handeye2` 包实现了两种经典的标定模式，其核心逻辑分为**TF数据采样**与**OpenCV矩阵求解**两部分。

**1. TF数据采样逻辑 (`handeye_sampler.cpp`)**
在标定过程中，系统需要记录多组机械臂位姿和相机识别到的标定板位姿。`easy_handeye2` 通过监听 ROS 2 的 TF 树来获取这些变换矩阵。针对不同的安装模式，采样逻辑有所不同：

```cpp
// 核心代码片段：根据标定模式获取机械臂位姿 (handeye_sampler.cpp)
if (params_.eye_on_hand) {
  // 眼在手上 (Eye-in-Hand)：获取 基座(base) -> 末端(effector) 的变换
  sample.robot_transform = tf_buffer_->lookupTransform(
    params_.robot_base_frame,
    params_.robot_effector_frame,
    tf2::TimePointZero,
    tf2::durationFromSec(5.0));
} else {
  // 眼在手外 (Eye-on-Base)：获取 末端(effector) -> 基座(base) 的变换
  // 注意：为了统一 AX=XB 求解模型，这里将 TF 变换方向进行了反转
  sample.robot_transform = tf_buffer_->lookupTransform(
    params_.robot_effector_frame,
    params_.robot_base_frame,
    tf2::TimePointZero,
    tf2::durationFromSec(5.0));
}

// 获取相机视觉位姿：相机(camera) -> 标定板(marker) 的变换
sample.optical_transform = tf_buffer_->lookupTransform(
  params_.tracking_base_frame,
  params_.tracking_marker_frame,
  tf2::TimePointZero,
  tf2::durationFromSec(5.0));
```
*解析*：对于“眼在手外”模式，求解的是 $AX=XB$ 方程，其中 $A$ 是机械臂末端在不同位姿间的相对变换，$B$ 是相机坐标系下标定板的相对变换。为了统一数学模型，源码中特意将“眼在手外”的机械臂 TF 变换方向进行了反转（即 `effector -> base`），从而复用同一套求解算法。

**2. 标定算法求解 (`calibration_backend_opencv.cpp`)**
在收集到足够的样本（通常10-15组）后，`easy_handeye2` 会调用 OpenCV 的 `cv::calibrateHandEye` 函数进行求解。该后端支持多种经典标定算法：

```cpp
// 核心代码片段：OpenCV标定算法映射 (calibration_backend_opencv.cpp)
cv::HandEyeCalibrationMethod CalibrationBackendOpenCV::getOpenCVMethod() const
{
  switch (current_algorithm_) {
    case CalibrationAlgorithm::TSAI_LENZ:
      return cv::CALIB_HAND_EYE_TSAI;       // Tsai-Lenz 算法（默认，速度快）
    case CalibrationAlgorithm::PARK:
      return cv::CALIB_HAND_EYE_PARK;       // Park 算法
    case CalibrationAlgorithm::HORAUD:
      return cv::CALIB_HAND_EYE_HORAUD;     // Horaud 算法
    case CalibrationAlgorithm::ANDREFF:
      return cv::CALIB_HAND_EYE_ANDREFF;    // Andreff 算法
    case CalibrationAlgorithm::DANIILIDIS:
      return cv::CALIB_HAND_EYE_DANIILIDIS; // Daniilidis 算法（对偶四元数，精度高）
    default:
      return cv::CALIB_HAND_EYE_TSAI;
  }
}
```
*解析*：
- **Tsai-Lenz**：基于轴角表示法，分别求解旋转和平移，计算速度快，是系统的默认算法。
- **Daniilidis**：基于对偶四元数，将旋转和平移结合在一起同时求解，通常在噪声较大的情况下能提供更高的精度。
系统会将 ROS 的 `geometry_msgs::msg::Transform` 转换为 OpenCV 的旋转矩阵（`cv::Mat`）和平移向量，传入上述算法求解出最终的静态 TF 变换矩阵（即相机与机械臂之间的固定位姿关系），并保存为 YAML 配置文件供后续抓取任务加载。

#### 3.2.2 代码节写：基于OpenCV的目标检测与位姿估计，逆运动学抓取规划

**1. 基于OpenCV的目标检测与位姿估计**
在 `capture_image` 包中，`rgb_detect_pose_node.cpp` 实现了基于颜色的目标检测与3D位姿估计。核心逻辑是通过颜色阈值过滤提取目标轮廓，并结合相机内参估算深度：

```cpp
// 核心代码片段：颜色阈值过滤与轮廓提取
cv::Mat hsv_image, mask;
cv::cvtColor(cv_image->image, hsv_image, cv::COLOR_BGR2HSV);

// 应用HSV阈值生成掩码
cv::inRange(hsv_image, 
            cv::Scalar(threshold.h_min, threshold.s_min, threshold.v_min),
            cv::Scalar(threshold.h_max, threshold.s_max, threshold.v_max), 
            mask);

// 寻找外部轮廓
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// 针对最大轮廓计算最小外接圆
cv::Point2f center;
float radius;
cv::minEnclosingCircle(contours[0], center, radius);

// 结合相机内参（ost.yaml）和物体真实物理尺寸估算深度(Z)
double focal_length = camera_matrix_.at<double>(0, 0);
double depth = (circle_real_diameter_ * focal_length) / (radius * 2.0);

// 计算物体在相机坐标系下的3D坐标 (X, Y, Z)
double x = (center.x - camera_matrix_.at<double>(0, 2)) * depth / focal_length;
double y = (center.y - camera_matrix_.at<double>(1, 2)) * depth / focal_length;
```
该节点通过相机内参和目标在图像中的像素大小，利用相似三角形原理估算出目标的深度，进而发布 `object_pose` 并广播TF变换。

**2. 逆运动学抓取规划与底层控制**
在获取到目标在基座坐标系下的位姿后，MoveIt 2 会进行逆运动学（IK）求解，生成关节空间的轨迹。`arm_controller.py` 负责接收这些关节角度并驱动硬件：

```python
# 核心代码片段：接收关节角度并转换为舵机PWM指令
def listener_callback(self, msg):
    # msg.data 包含了MoveIt规划出的6个关节的目标角度（弧度）
    angles = msg.data
    for i in range(6):
        # 弧度转角度，并应用YAML配置文件中的舵机偏差补偿
        angle_deg = math.degrees(angles[i]) + self.servo_offsets[f'servo_{i}']
        
        # 将物理角度映射为舵机的PWM信号 (500-2500)
        pwm = self.calculate_pwm(angle_deg, angle_min=-135, angle_max=135)
        
        # 拼接串口通信协议指令并发送给下位机
        # 协议格式: #<id>P<pwm>T<time>!
        self.send_servo_command(self.ser, servo_id=i, angle=angle_deg, time=1000)
```
该控制器不仅实现了角度到PWM的线性映射，还引入了 `servo_calibration.yaml` 进行软件层面的零点校准，有效消除了机械装配带来的误差。

## 4. 典型案例使用说明

### 4.1 视觉抓取综合案例
本案例展示了如何将上述模块串联，实现“识别-定位-抓取”的完整流程。

**操作步骤**：
1. **启动底层驱动与MoveIt**：
   ```bash
   ros2 launch A1_moveit_config demo.launch.py
   ros2 run arm_controller arm_controller_node
   ```
2. **启动视觉识别节点**：
   ```bash
   ros2 launch capture_image capture_image.launch.py
   ```
3. **发布静态手眼标定TF**（假设已完成标定）：
   ```bash
   ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll base_link camera_link
   ```
4. **执行抓取任务**：
   视觉节点识别到目标后，会向TF树中广播 `object` 坐标系。此时可以通过编写的抓取动作客户端，读取 `base_link` 到 `object` 的变换，将其作为目标位姿发送给 MoveIt 2 进行路径规划与执行。
