/******************************************************************************
 * Copyright 2020-2025, zhangsai. All Rights Reserved.
 *****************************************************************************/

#include "capture_image.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<capture_image::CaptureImage>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}