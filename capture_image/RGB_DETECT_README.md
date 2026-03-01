# RGB颜色检测与姿态估计节点

## 功能描述

这个ROS2节点通过颜色识别来检测圆形物体，并计算其在机器人坐标系下的3D位置。

### 主要功能

1. **颜色识别**：支持红色(R)、绿色(G)、蓝色(B)三种颜色的识别
2. **圆形检测**：通过轮廓分析和圆度计算识别圆形物体
3. **3D位置计算**：根据圆形的真实尺寸和图像尺寸计算物体的3D坐标
4. **TF发布**：将检测到的物体位置通过TF2发布到相机坐标系
5. **颜色类型发布**：通过`std_msgs/String`消息发布检测到的颜色类型("r", "g", "b")
6. **实时参数调整**：提供GUI滑动条界面，可实时调整HSV颜色阈值

## 订阅话题

- `/camera/image_rect` (sensor_msgs/Image): 校正后的相机图像

## 发布话题

- `/rgb_detection/image_raw` (sensor_msgs/Image): 带有检测结果标注的图像
- `/rgb_detection/compressed` (sensor_msgs/CompressedImage): 压缩图像
- `/rgb_detection/color_type` (std_msgs/String): 检测到的颜色类型 ("r", "g", "b")

## 发布TF

- `detected_object_r`: 红色物体的位置 (相对于camera_frame)
- `detected_object_g`: 绿色物体的位置 (相对于camera_frame)
- `detected_object_b`: 蓝色物体的位置 (相对于camera_frame)

## 参数说明

- `circle_real_diameter_mm` (double, 默认: 50.0): 圆形物体的真实直径(毫米)
- `camera_frame` (string, 默认: "camera_link"): 相机坐标系名称
- `show_gui` (bool, 默认: true): 是否显示GUI窗口和滑动条
- `jpeg_quality` (int, 默认: 80): JPEG压缩质量 (1-100)

## 编译

```bash
cd ~/marm_ws
colcon build --packages-select capture_image
source install/setup.bash
```

## 使用方法

### 1. 启动节点（使用默认参数）

```bash
ros2 launch capture_image rgb_detect_pose.launch.py
```

### 2. 启动节点（自定义参数）

```bash
ros2 launch capture_image rgb_detect_pose.launch.py \
    circle_real_diameter_mm:=60.0 \
    camera_frame:=camera_color_optical_frame \
    show_gui:=true
```

### 3. 直接运行节点

```bash
ros2 run capture_image rgb_detect_pose_node
```

## 颜色阈值调整

启动节点后，会出现一个名为"RGB Detection Control"的窗口，包含以下滑动条：

- **Color Select**: 选择要调整的颜色 (0: Red1, 1: Red2, 2: Green, 3: Blue)
- **H Min/H Max**: 色调(Hue)的最小值和最大值 (0-180)
- **S Min/S Max**: 饱和度(Saturation)的最小值和最大值 (0-255)
- **V Min/V Max**: 明度(Value)的最小值和最大值 (0-255)

### 默认颜色阈值

#### 红色 (Red)
- 范围1: H(0-10), S(100-255), V(100-255)
- 范围2: H(170-180), S(100-255), V(100-255)
- 说明：HSV色彩空间中红色在0和180附近，因此需要两个范围

#### 绿色 (Green)
- H(40-80), S(100-255), V(100-255)

#### 蓝色 (Blue)
- H(100-130), S(100-255), V(100-255)

### HSV颜色空间说明

- **H (Hue)**: 色调，表示颜色类型
  - 红色: 0-10, 170-180
  - 绿色: 40-80
  - 蓝色: 100-130
  
- **S (Saturation)**: 饱和度，颜色的纯度
  - 值越高，颜色越鲜艳
  - 值越低，颜色越接近灰色
  
- **V (Value)**: 明度，颜色的亮度
  - 值越高，颜色越亮
  - 值越低，颜色越暗

## 调试技巧

### 1. 调整颜色阈值

如果检测效果不好，可以按以下步骤调整：

1. 在"RGB Detection Control"窗口中选择对应的颜色
2. 先放宽阈值范围（降低S Min和V Min，提高H Max、S Max、V Max）
3. 观察检测效果
4. 逐步收紧阈值，直到只检测到目标颜色

### 2. 检查相机参数

确保`ost.yaml`文件包含正确的相机内参：
```yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

### 3. 验证圆形尺寸

确保参数`circle_real_diameter_mm`与实际圆形物体的直径一致，这对3D位置计算至关重要。

### 4. 查看检测结果

使用RViz查看TF变换：
```bash
ros2 run rviz2 rviz2
```
添加TF显示，可以看到`detected_object_r/g/b`相对于`camera_frame`的位置。

### 5. 监控颜色类型

```bash
ros2 topic echo /rgb_detection/color_type
```

## 工作原理

### 1. 颜色识别流程

1. 将BGR图像转换为HSV色彩空间
2. 对每种颜色应用HSV阈值，生成二值掩码
3. 使用形态学操作去除噪声
4. 查找轮廓并筛选

### 2. 圆形检测

1. 计算轮廓面积，过滤小面积
2. 使用最小外接圆拟合轮廓
3. 计算圆度 = 4π × 面积 / 周长²
4. 圆度 > 0.7 认为是圆形

### 3. 3D位置计算

使用针孔相机模型：

```
Z = (fx × real_diameter) / pixel_diameter
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
```

其中：
- (u, v): 圆心在图像中的像素坐标
- (cx, cy): 相机主点
- fx, fy: 相机焦距
- Z: 物体到相机的距离（深度）
- X, Y: 物体在相机坐标系中的横向和纵向位置

## 注意事项

1. 需要先运行相机标定，确保`ost.yaml`文件存在且正确
2. 确保光照条件良好，避免过亮或过暗
3. 圆形物体表面应尽量均匀，避免反光
4. 圆形物体应与背景颜色有明显区别
5. 单次只会检测第一个满足条件的圆形物体

## 故障排除

### 问题1: 无法检测到颜色

**解决方案**:
- 检查光照条件
- 使用滑动条放宽颜色阈值
- 确保物体颜色纯度足够高

### 问题2: 3D位置不准确

**解决方案**:
- 验证相机标定参数是否正确
- 确认`circle_real_diameter_mm`参数与实际尺寸一致
- 检查圆形在图像中是否完整可见

### 问题3: 检测到多个圆形或误检测

**解决方案**:
- 收紧颜色阈值
- 调整圆度阈值（修改代码中的0.7参数）
- 改善拍摄环境，减少干扰

## 扩展开发

如果需要添加更多颜色或修改检测逻辑，可以修改代码中的以下部分：

### 添加新颜色

在`initializeColorThresholds()`函数中添加：

```cpp
// 黄色
color_thresholds_.push_back({20, 40, 100, 255, 100, 255, "Yellow", "y"});
```

### 修改圆度阈值

在`image_callback()`函数中修改：

```cpp
if (circularity > 0.7 && radius > 10) {  // 修改0.7为其他值
```

### 修改最小面积阈值

```cpp
if (area < 100) continue;  // 修改100为其他值
```

## 性能优化建议

1. 降低图像分辨率可以提高处理速度
2. 减少JPEG压缩质量可以降低网络带宽占用
3. 关闭GUI（设置`show_gui:=false`）可以提高性能
4. 仅检测单一颜色可以注释掉其他颜色的检测代码
