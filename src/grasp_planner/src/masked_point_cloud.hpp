#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Geometry>

class MoveGroupInterfaceNode;

class MaskedPointCloud : public rclcpp::Node
{
public:
    MaskedPointCloud(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void setup();

    std::tuple<cv::Mat, cv::Mat, sensor_msgs::msg::CameraInfo> getFrame();
    bool hasFrame() const { return frame_ready_; }

private:
    void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void frameCallback(sensor_msgs::msg::Image::ConstSharedPtr colorMsg, sensor_msgs::msg::Image::ConstSharedPtr depthMsg, sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo);
    void publishCloud(const cv::Mat &depth, const sensor_msgs::msg::CameraInfo &caminfo);

private:
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> caminfo_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_, depth_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    using SyncPolicy = typename message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    std::unique_ptr<Synchronizer> sync_;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<image_transport::Subscriber> mask_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    cv::Mat rgb_, depth_, mask_, depth_masked_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_ = nullptr;

    std::string camera_info_topic_, rgb_topic_, depth_topic_, mask_topic_, pcl_topic_, target_frame_;
    std::mutex frame_mutex_;
    std::atomic<bool> frame_ready_{false};
    uint depth_threshold_;
    Eigen::Isometry3d camera_transform_ ;
    bool has_camera_transform_ = false ;
};