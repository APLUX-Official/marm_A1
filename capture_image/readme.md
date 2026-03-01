## camera标定
ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.01 image:=/camera/image_raw camera:=/camera --no-service-check

## 相机标定结果
标定结束后，县级save，然后标定的数据存放在/tmp/calibration中，将解压后的ost.yaml文件放到../capture_image目录下。


# 启动相机
ros2 launch capture_image capture_image.launch.py

# 启动aruco识别
ros2 run capture_image aruco_object_pose_node


# 启动红色物体识别
ros2 run capture_image red_object_pose_node