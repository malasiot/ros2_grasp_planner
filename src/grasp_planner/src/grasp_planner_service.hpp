#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "grasp_planner_interfaces/srv/grasp_planner.hpp"
#include "grasp_planner_interfaces/srv/grasp_net.hpp"
#include "cv_bridge/cv_bridge.h"

class MoveGroupInterfaceNode ;

class GraspPlannerService:  public rclcpp::Node {
public:
    GraspPlannerService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    void setup(const std::shared_ptr<MoveGroupInterfaceNode> &mgi);

 
private:

 void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
 void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
 void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr colorMsg, const sensor_msgs::msg::Image::ConstSharedPtr depthMsg) ;

private:
   
    using GraspPlannerSrv = grasp_planner_interfaces::srv::GraspPlanner ;
    using GraspNet = grasp_planner_interfaces::srv::GraspNet;

    rclcpp::Service<GraspPlannerSrv>::SharedPtr service_ ;
     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
     message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_, depth_sub_ ;
     
     using SyncPolicy =  typename message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ;
     using Synchronizer = message_filters::Synchronizer<SyncPolicy> ;
     std::unique_ptr<Synchronizer> sync_ ;
     std::shared_ptr<image_transport::ImageTransport> image_transport_;
     std::shared_ptr<image_transport::Subscriber> mask_sub_;
     std::shared_ptr<rclcpp::Client<GraspNet>> graspnet_client_ ;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> grasps_rviz_pub_ ;

    void plan(const std::shared_ptr<GraspPlannerSrv::Request> request, std::shared_ptr<GraspPlannerSrv::Response> response) ;
   
    cv::Mat rgb_, depth_, mask_ ;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_ = nullptr ;

    std::string camera_info_topic_, rgb_topic_, depth_topic_, mask_topic_ ;
    std::mutex frame_mutex_, mask_mutex_ ;
    std::atomic<bool> frame_ready_ {false} ;
    std::shared_ptr<MoveGroupInterfaceNode> move_group_interface_ ;
  
};